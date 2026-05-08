#include "teleop_viewer/config.h"
#include "teleop_viewer/ik_solver.h"
#include "teleop_viewer/scene.h"

#include "imgui.h"
#include "ImGuizmo.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <geometry_msgs/PoseStamped.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ros/ros.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using omnilink::teleop_viewer::IkChainStatus;
using omnilink::teleop_viewer::IkSolver;
using omnilink::teleop_viewer::IkSolveStats;
using omnilink::teleop_viewer::OrbitCamera;
using omnilink::teleop_viewer::RobotScene;
using omnilink::teleop_viewer::ViewerConfig;

namespace {

    const char* kMeshVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;
void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoords = aTexCoords;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

    const char* kMeshFragmentShader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 diffuseColor;
uniform bool hasTexture;
uniform sampler2D texture_diffuse1;
out vec4 color;
void main() {
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * diffuseColor;
    vec3 ambient = 0.58 * diffuseColor;
    vec3 result = ambient + diffuse;
    if (hasTexture) {
        result *= texture(texture_diffuse1, TexCoords).rgb;
    }
    color = vec4(result, 1.0);
}
)";

    const char* kLineVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
uniform mat4 view;
uniform mat4 projection;
out vec3 Color;
void main() {
    Color = aColor;
    gl_Position = projection * view * vec4(aPos, 1.0);
}
)";

    const char* kLineFragmentShader = R"(
#version 330 core
in vec3 Color;
out vec4 FragColor;
void main() {
    FragColor = vec4(Color, 1.0);
}
)";

    GLuint compileShader(GLenum type, const char* src) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);
        GLint ok = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (ok != GL_TRUE) {
            char log[1024];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            std::cerr << "Shader compile failed: " << log << std::endl;
        }
        return shader;
    }

    GLuint createProgram(const char* vs_src, const char* fs_src) {
        GLuint vs      = compileShader(GL_VERTEX_SHADER, vs_src);
        GLuint fs      = compileShader(GL_FRAGMENT_SHADER, fs_src);
        GLuint program = glCreateProgram();
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);
        GLint ok = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (ok != GL_TRUE) {
            char log[1024];
            glGetProgramInfoLog(program, sizeof(log), nullptr, log);
            std::cerr << "Program link failed: " << log << std::endl;
        }
        glDeleteShader(vs);
        glDeleteShader(fs);
        return program;
    }

    struct LineVertex {
        glm::vec3 p;
        glm::vec3 c;
    };

    class LineRenderer {
       public:
        void init() {
            glGenVertexArrays(1, &vao_);
            glGenBuffers(1, &vbo_);
        }

        void draw(GLuint shader, const std::vector<LineVertex>& vertices, const glm::mat4& view, const glm::mat4& proj, float line_width) {
            if (vertices.empty()) {
                return;
            }
            glUseProgram(shader);
            glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));

            glBindVertexArray(vao_);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(LineVertex)), vertices.data(), GL_DYNAMIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)offsetof(LineVertex, p));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)offsetof(LineVertex, c));

            glLineWidth(line_width);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
            glBindVertexArray(0);
        }

        ~LineRenderer() {
            if (vbo_ != 0)
                glDeleteBuffers(1, &vbo_);
            if (vao_ != 0)
                glDeleteVertexArrays(1, &vao_);
        }

       private:
        GLuint vao_ = 0;
        GLuint vbo_ = 0;
    };

    struct ViewerState {
        bool show_axes             = true;
        bool show_world_axes       = true;
        bool show_revolute_only    = true;
        bool lock_base             = true;
        bool show_non_revolute     = false;
        float axis_length          = 0.12f;
        float axis_line_width      = 2.0f;
        float world_axis_length    = 0.4f;
        float grid_size            = 4.0f;
        int grid_count             = 40;
        float panel_width          = 430.0f;
        float joint_section_height = 260.0f;
        bool panel_resize_active   = false;
        char joint_filter[128]     = {0};
        char tf_filter[128]        = {0};
        int selected_joint         = -1;
        std::unordered_map<std::string, float> pose_snapshot;
    };

    struct IkState {
        struct MarkerTarget {
            bool initialized  = false;
            glm::vec3 pos     = glm::vec3(0.0f);
            glm::vec3 rpy_deg = glm::vec3(0.0f);
        };

        IkSolver solver;
        std::vector<IkChainStatus> chains;
        std::vector<MarkerTarget> marker_targets;
        std::string solve_mode        = "single_chain";  // single_chain | full_body
        std::string full_body_backend = "flex_ik";       // flex_ik | wbc_chain_ik
        int full_body_iterations      = 3;
        int selected_chain            = 0;
        bool marker_initialized       = false;
        bool lock_orientation         = false;
        float marker_pos[3]           = {0.0f, 0.0f, 0.0f};
        float marker_rpy_deg[3]       = {0.0f, 0.0f, 0.0f};
        std::string last_status;
        bool drag_mode_rotate             = false;
        bool dragging_marker              = false;
        bool marker_hovered               = false;
        bool left_mouse_prev              = false;
        int active_axis                   = -1;  // 0:x 1:y 2:z
        bool active_rotate                = false;
        float translate_sensitivity       = 0.35f;
        float rotate_sensitivity          = 0.20f;
        int drag_mode                     = 0;  // 0:view-plane move, 1/2/3 move x/y/z, 4/5/6 rotate x/y/z
        float drag_prev_x                 = 0.0f;
        float drag_prev_y                 = 0.0f;
        bool drag_prev_valid              = false;
        int gizmo_operation               = 0;  // 0 translate, 1 rotate, 2 universal
        bool gizmo_was_using              = false;
        bool gizmo_pose_dirty             = false;
        bool gizmo_drag_interacted        = false;
        bool gizmo_drag_position_only     = true;
        bool gizmo_was_over               = false;
        bool gizmo_world_mode             = true;
        bool realtime_ik_during_drag      = true;
        float realtime_ik_hz              = 30.0f;
        double last_realtime_ik_apply_sec = -1.0;
        float gizmo_size_clip_space       = 0.22f;
        bool translate_snap_enabled       = false;
        bool rotate_snap_enabled          = false;
        float translate_snap_step_m       = 0.01f;
        float rotate_snap_step_deg        = 5.0f;

        // External RViz interactive marker input (PoseStamped).
        bool use_external_target          = true;
        bool external_target_received     = false;
        bool external_target_dirty        = false;
        bool external_target_position_only = true;
        std::string external_target_topic = "/teleop_gui/ik_target_pose";
        std::string external_target_expected_frame;
        std::string external_target_last_frame;
        glm::vec3 external_target_pos        = glm::vec3(0.0f);
        glm::quat external_target_quat       = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // w, x, y, z
        double external_target_last_recv_sec = 0.0;
    };

    struct PoseKeyframe {
        double t = 0.0;
        std::unordered_map<std::string, float> joints;
    };

    struct DebugPlaybackState {
        std::vector<PoseKeyframe> keyframes;
        bool playing     = false;
        bool loop        = true;
        float play_speed = 1.0f;
        float play_time  = 0.0f;
    };

    float g_scroll_delta = 0.0f;

    void ScrollCallback(GLFWwindow*, double, double yoffset) {
        g_scroll_delta += static_cast<float>(yoffset);
    }

    std::string getUrdfPathFromArgs(int argc, char** argv) {
        if (argc <= 1) {
            return "config/robot_viewer.yaml";
        }
        return argv[1];
    }

    bool fileExists(const std::string& path) {
        std::error_code ec;
        return std::filesystem::exists(path, ec);
    }

    void setupFonts(const ViewerConfig& cfg) {
        ImGuiIO& io           = ImGui::GetIO();
        float font_size       = std::max(12.0f, cfg.ui.cjk_font_size);
        const ImWchar* ranges = io.Fonts->GetGlyphRangesChineseFull();

        std::string loaded_font_path;
        ImFontConfig font_cfg;
        font_cfg.OversampleH = 2;
        font_cfg.OversampleV = 1;
        font_cfg.PixelSnapH  = true;

        std::vector<std::string> font_candidates;
        if (!cfg.ui.cjk_font_path.empty()) {
            font_candidates.push_back(cfg.ui.cjk_font_path);
        }
        font_candidates.push_back("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
        font_candidates.push_back("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc");
        font_candidates.push_back("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf");
        font_candidates.push_back("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc");
        font_candidates.push_back("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc");

        for (const auto& path : font_candidates) {
            if (!fileExists(path)) {
                continue;
            }
            if (io.Fonts->AddFontFromFileTTF(path.c_str(), font_size, &font_cfg, ranges)) {
                loaded_font_path = path;
                std::cout << "[robot_kinematic_viewer] Loaded CJK font: " << loaded_font_path << " (size=" << font_size << ")" << std::endl;
                return;
            }
        }
        io.Fonts->AddFontDefault();
        std::cerr << "[robot_kinematic_viewer] No CJK font found. Chinese text may show as '?'. "
                  << "Please set ui.cjk_font_path in config." << std::endl;
    }

    glm::mat4 markerWorldMatrix(const glm::vec3& pos, const glm::vec3& rpyDeg) {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
        m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[0]), glm::vec3(1.0f, 0.0f, 0.0f));
        m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[1]), glm::vec3(0.0f, 1.0f, 0.0f));
        m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[2]), glm::vec3(0.0f, 0.0f, 1.0f));
        return m;
    }

    void appendMarkerAxes(std::vector<LineVertex>* out, const glm::vec3& pos, const glm::vec3& rpy_deg, float axis_len, bool selected) {
        if (out == nullptr) {
            return;
        }
        const glm::mat4 rot    = markerWorldMatrix(glm::vec3(0.0f), rpy_deg);
        const glm::vec3 x_axis = glm::normalize(glm::vec3(rot * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
        const glm::vec3 y_axis = glm::normalize(glm::vec3(rot * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
        const glm::vec3 z_axis = glm::normalize(glm::vec3(rot * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
        const float scale      = selected ? 1.0f : 0.75f;
        const glm::vec3 cx     = selected ? glm::vec3(1.0f, 0.25f, 0.25f) : glm::vec3(0.85f, 0.45f, 0.45f);
        const glm::vec3 cy     = selected ? glm::vec3(0.25f, 1.0f, 0.25f) : glm::vec3(0.45f, 0.85f, 0.45f);
        const glm::vec3 cz     = selected ? glm::vec3(0.25f, 0.55f, 1.0f) : glm::vec3(0.45f, 0.65f, 0.85f);

        out->push_back({pos, cx});
        out->push_back({pos + x_axis * axis_len * scale, cx});
        out->push_back({pos, cy});
        out->push_back({pos + y_axis * axis_len * scale, cy});
        out->push_back({pos, cz});
        out->push_back({pos + z_axis * axis_len * scale, cz});
    }

    void appendCircle(std::vector<LineVertex>* out, const glm::vec3& center, const glm::vec3& n, float r, const glm::vec3& c, int seg) {
        if (out == nullptr || seg < 8) {
            return;
        }
        glm::vec3 normal = glm::normalize(n);
        glm::vec3 helper = std::fabs(normal.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 u      = glm::normalize(glm::cross(normal, helper));
        glm::vec3 v      = glm::normalize(glm::cross(normal, u));
        for (int i = 0; i < seg; ++i) {
            float t0     = (2.0f * 3.1415926f * static_cast<float>(i)) / static_cast<float>(seg);
            float t1     = (2.0f * 3.1415926f * static_cast<float>(i + 1)) / static_cast<float>(seg);
            glm::vec3 p0 = center + r * (std::cos(t0) * u + std::sin(t0) * v);
            glm::vec3 p1 = center + r * (std::cos(t1) * u + std::sin(t1) * v);
            out->push_back({p0, c});
            out->push_back({p1, c});
        }
    }

    glm::vec2 worldToScreen(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, int viewport_w, int viewport_h, bool* ok) {
        glm::vec4 clip = proj * view * glm::vec4(p, 1.0f);
        if (std::fabs(clip.w) < 1e-6f) {
            if (ok)
                *ok = false;
            return glm::vec2(-1.0f);
        }
        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        if (ok)
            *ok = true;
        return glm::vec2((ndc.x * 0.5f + 0.5f) * static_cast<float>(viewport_w),
                         (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(viewport_h));
    }

    float distancePointToSegment2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
        glm::vec2 ab = b - a;
        float ab2    = glm::dot(ab, ab);
        if (ab2 < 1e-6f) {
            return glm::length(p - a);
        }
        float t        = glm::clamp(glm::dot(p - a, ab) / ab2, 0.0f, 1.0f);
        glm::vec2 proj = a + t * ab;
        return glm::length(p - proj);
    }

}  // namespace

int main(int argc, char** argv) {
    if (!ros::isInitialized()) {
        ros::init(argc, argv, "robot_kinematic_viewer", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }

    std::string config_or_urdf = getUrdfPathFromArgs(argc, argv);
    ViewerConfig cfg           = ViewerConfig::LoadFromFile(config_or_urdf);
    std::string urdf_path      = cfg.robot.urdf_path;
    if (config_or_urdf.size() > 5 && config_or_urdf.substr(config_or_urdf.size() - 5) == ".urdf") {
        urdf_path = config_or_urdf;
    }

    if (!glfwInit()) {
        std::cerr << "glfwInit failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(cfg.window.width, cfg.window.height, "Robot Kinematic Debug Viewer", nullptr, nullptr);
    if (!window) {
        std::cerr << "create window failed\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetScrollCallback(window, ScrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "glad init failed\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    setupFonts(cfg);
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    GLuint mesh_shader = createProgram(kMeshVertexShader, kMeshFragmentShader);
    GLuint line_shader = createProgram(kLineVertexShader, kLineFragmentShader);
    LineRenderer line_renderer;
    line_renderer.init();

    RobotScene scene;
    if (!scene.loadURDF(urdf_path)) {
        std::cerr << "Failed to load URDF: " << urdf_path << "\n";
        return 1;
    }

    OrbitCamera camera;
    camera.distance     = cfg.camera.distance;
    camera.yaw          = cfg.camera.yaw;
    camera.pitch        = cfg.camera.pitch;
    camera.target       = cfg.camera.target;
    camera.rotate_speed = cfg.camera.rotate_speed;
    camera.zoom_scale   = cfg.camera.zoom_scale;
    camera.dolly_scale  = cfg.camera.dolly_scale;
    camera.pan_scale    = cfg.camera.pan_scale;
    camera.min_distance = cfg.camera.min_distance;
    camera.max_distance = cfg.camera.max_distance;

    ViewerState ui_state;
    ui_state.lock_base = cfg.ui.fix_base_like_mujoco;
    scene.setFixedBaseMode(ui_state.lock_base);

    ros::NodeHandle ros_nh_private("~");
    std::string ikModeParam;
    std::string fullBodyBackendParam;
    int fullBodyIterationsParam = cfg.ik.full_body_iterations;
    if (ros_nh_private.getParam("ik_mode", ikModeParam)) {
        cfg.ik.mode = ikModeParam;
    }
    if (ros_nh_private.getParam("ik_full_body_backend", fullBodyBackendParam)) {
        cfg.ik.full_body_backend = fullBodyBackendParam;
    }
    if (ros_nh_private.getParam("ik_full_body_iterations", fullBodyIterationsParam)) {
        cfg.ik.full_body_iterations = std::max(1, fullBodyIterationsParam);
    }

    IkState ik_state;
    DebugPlaybackState playback_state;
    {
        ik_state.solve_mode = cfg.ik.mode;
        std::transform(ik_state.solve_mode.begin(), ik_state.solve_mode.end(), ik_state.solve_mode.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.solve_mode != "single_chain" && ik_state.solve_mode != "full_body") {
            ik_state.solve_mode = "single_chain";
        }
        ik_state.full_body_backend = cfg.ik.full_body_backend;
        std::transform(ik_state.full_body_backend.begin(), ik_state.full_body_backend.end(), ik_state.full_body_backend.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.full_body_backend != "flex_ik" && ik_state.full_body_backend != "wbc_chain_ik") {
            ik_state.full_body_backend = "flex_ik";
        }
        ik_state.full_body_iterations = cfg.ik.full_body_iterations;
        ik_state.solver.setFullBodyBackend(ik_state.full_body_backend);
        ik_state.solver.initialize(urdf_path, cfg.ik.chains);
        ik_state.chains.clear();
        for (int i = 0; i < ik_state.solver.chainCount(); ++i) {
            ik_state.chains.push_back(ik_state.solver.chainStatus(i));
        }
        ros_nh_private.param<int>("ik_selected_chain", ik_state.selected_chain, ik_state.selected_chain);
        if (!ik_state.chains.empty()) {
            ik_state.selected_chain = std::clamp(ik_state.selected_chain, 0, static_cast<int>(ik_state.chains.size()) - 1);
        }
        ik_state.marker_targets.resize(ik_state.chains.size());
    }

    ros_nh_private.param<std::string>("ik_target_pose_topic", ik_state.external_target_topic, ik_state.external_target_topic);
    ros_nh_private.param<std::string>("ik_target_pose_frame", ik_state.external_target_expected_frame,
                                      ik_state.external_target_expected_frame);
    ros_nh_private.param<bool>("enable_external_ik_target", ik_state.use_external_target, ik_state.use_external_target);
    ros_nh_private.param<bool>("external_ik_target_position_only", ik_state.external_target_position_only,
                               ik_state.external_target_position_only);

    ros::Subscriber external_target_sub = ros_nh_private.subscribe<geometry_msgs::PoseStamped>(
        ik_state.external_target_topic, 20, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            if (msg == nullptr) {
                return;
            }
            const std::string frame = msg->header.frame_id;
            if (!ik_state.external_target_expected_frame.empty() && frame != ik_state.external_target_expected_frame) {
                return;
            }
            ik_state.external_target_pos = glm::vec3(static_cast<float>(msg->pose.position.x), static_cast<float>(msg->pose.position.y),
                                                     static_cast<float>(msg->pose.position.z));
            ik_state.external_target_quat =
                glm::quat(static_cast<float>(msg->pose.orientation.w), static_cast<float>(msg->pose.orientation.x),
                          static_cast<float>(msg->pose.orientation.y), static_cast<float>(msg->pose.orientation.z));
            if (glm::length(ik_state.external_target_quat) > 1e-6f) {
                ik_state.external_target_quat = glm::normalize(ik_state.external_target_quat);
            } else {
                ik_state.external_target_quat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }
            ik_state.external_target_last_frame    = frame;
            ik_state.external_target_last_recv_sec = ros::Time::now().toSec();
            ik_state.external_target_received      = true;
            ik_state.external_target_dirty         = true;
        });
    (void)external_target_sub;

    double prev_x         = 0.0;
    double prev_y         = 0.0;
    bool first_mouse      = true;
    double last_frame_sec = glfwGetTime();

    auto ensureMarkerTargetInitialized = [&](int chain_index) -> bool {
        if (chain_index < 0 || chain_index >= static_cast<int>(ik_state.marker_targets.size())) {
            return false;
        }
        auto& target = ik_state.marker_targets[chain_index];
        if (target.initialized) {
            return true;
        }
        glm::vec3 tip_pos(0.0f);
        glm::vec3 tip_rpy(0.0f);
        if (!ik_state.solver.fetchTipWorldPose(scene, chain_index, &tip_pos, &tip_rpy)) {
            return false;
        }
        target.pos         = tip_pos;
        target.rpy_deg     = glm::vec3(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
        target.initialized = true;
        return true;
    };

    auto loadActiveMarkerFromTarget = [&]() -> bool {
        if (!ensureMarkerTargetInitialized(ik_state.selected_chain)) {
            return false;
        }
        const auto& target          = ik_state.marker_targets[ik_state.selected_chain];
        ik_state.marker_pos[0]      = target.pos.x;
        ik_state.marker_pos[1]      = target.pos.y;
        ik_state.marker_pos[2]      = target.pos.z;
        ik_state.marker_rpy_deg[0]  = target.rpy_deg.x;
        ik_state.marker_rpy_deg[1]  = target.rpy_deg.y;
        ik_state.marker_rpy_deg[2]  = target.rpy_deg.z;
        ik_state.marker_initialized = true;
        return true;
    };

    auto saveActiveMarkerToTarget = [&]() {
        if (ik_state.selected_chain < 0 || ik_state.selected_chain >= static_cast<int>(ik_state.marker_targets.size())) {
            return;
        }
        auto& target       = ik_state.marker_targets[ik_state.selected_chain];
        target.pos         = glm::vec3(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
        target.rpy_deg     = glm::vec3(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
        target.initialized = true;
    };

    if (!ik_state.chains.empty()) {
        loadActiveMarkerFromTarget();
    }

    while (!glfwWindowShouldClose(window)) {
        if (ros::ok()) {
            ros::spinOnce();
        }
        glfwPollEvents();
        double now_sec = glfwGetTime();
        double dt_sec  = std::max(0.0, now_sec - last_frame_sec);
        last_frame_sec = now_sec;

        double x = 0.0, y = 0.0;
        glfwGetCursorPos(window, &x, &y);
        if (first_mouse) {
            prev_x      = x;
            prev_y      = y;
            first_mouse = false;
        }
        double dx = x - prev_x;
        double dy = y - prev_y;
        prev_x    = x;
        prev_y    = y;

        int fb_w = 0, fb_h = 0;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        float panel_min      = 280.0f;
        float panel_max      = std::max(panel_min, static_cast<float>(fb_w) - 320.0f);
        ui_state.panel_width = std::clamp(ui_state.panel_width, panel_min, panel_max);
        int panel_w          = static_cast<int>(ui_state.panel_width);
        int viewport_w       = std::max(1, fb_w - panel_w);
        int viewport_h       = std::max(1, fb_h);

        bool mouse_in_viewport = (x >= 0.0 && x < static_cast<double>(viewport_w) && y >= 0.0 && y < static_cast<double>(viewport_h));
        const bool block_camera_input = ui_state.panel_resize_active;
        if (mouse_in_viewport && !block_camera_input && !ik_state.dragging_marker && !ik_state.gizmo_was_using &&
            !ik_state.gizmo_was_over) {
            bool left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            bool right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            bool shift  = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            if (left && !shift && !middle) {
                camera.rotate(static_cast<float>(dx), static_cast<float>(dy));
            } else if (middle || (shift && left)) {
                camera.pan(static_cast<float>(dx), static_cast<float>(dy));
            } else if (right) {
                camera.dolly(static_cast<float>(-dy));
            }
            if (std::fabs(g_scroll_delta) > 1e-6f) {
                camera.zoom(g_scroll_delta);
                g_scroll_delta = 0.0f;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        glViewport(0, 0, viewport_w, viewport_h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.90f, 0.92f, 0.96f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (playback_state.playing && playback_state.keyframes.size() >= 2) {
            const float total = static_cast<float>(playback_state.keyframes.back().t);
            playback_state.play_time += static_cast<float>(dt_sec) * playback_state.play_speed;
            if (playback_state.loop && total > 1e-4f) {
                while (playback_state.play_time > total) {
                    playback_state.play_time -= total;
                }
            } else if (playback_state.play_time > total) {
                playback_state.play_time = total;
                playback_state.playing   = false;
            }

            size_t hi = 1;
            while (hi < playback_state.keyframes.size() && static_cast<float>(playback_state.keyframes[hi].t) < playback_state.play_time) {
                ++hi;
            }
            size_t lo      = (hi == 0) ? 0 : (hi - 1);
            hi             = std::min(hi, playback_state.keyframes.size() - 1);
            const auto& k0 = playback_state.keyframes[lo];
            const auto& k1 = playback_state.keyframes[hi];
            float t0       = static_cast<float>(k0.t);
            float t1       = static_cast<float>(k1.t);
            float alpha    = (t1 > t0 + 1e-6f) ? ((playback_state.play_time - t0) / (t1 - t0)) : 0.0f;
            alpha          = std::clamp(alpha, 0.0f, 1.0f);

            auto joints_now = scene.getJointInfos();
            for (const auto& j : joints_now) {
                auto it0 = k0.joints.find(j.name);
                auto it1 = k1.joints.find(j.name);
                if (it0 == k0.joints.end() || it1 == k1.joints.end()) {
                    continue;
                }
                float v = it0->second * (1.0f - alpha) + it1->second * alpha;
                scene.setJointPositionByName(j.name, v);
            }
        }

        scene.setFixedBaseMode(ui_state.lock_base);
        scene.updateTransforms();

        glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 view = camera.viewMatrix();
        glm::vec3 eye  = camera.eye();

        glUseProgram(mesh_shader);
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glm::vec3 light_pos = eye + glm::vec3(0.8f, 0.8f, 1.2f);
        glUniform3f(glGetUniformLocation(mesh_shader, "lightPos"), light_pos.x, light_pos.y, light_pos.z);
        glUniform3f(glGetUniformLocation(mesh_shader, "viewPos"), eye.x, eye.y, eye.z);
        scene.draw(mesh_shader);

        std::vector<LineVertex> axis_vertices;
        {
            float half = ui_state.grid_size;
            int count  = std::max(2, ui_state.grid_count);
            glm::vec3 grid_col(0.72f, 0.76f, 0.82f);
            for (int i = 0; i <= count; ++i) {
                float t = -half + 2.0f * half * (static_cast<float>(i) / static_cast<float>(count));
                axis_vertices.push_back({glm::vec3(-half, t, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(half, t, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(t, -half, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(t, half, 0.0f), grid_col});
            }
        }
        if (ui_state.show_world_axes) {
            glm::vec3 o(0.0f);
            float l = ui_state.world_axis_length;
            axis_vertices.push_back({o, glm::vec3(1, 0, 0)});
            axis_vertices.push_back({o + glm::vec3(l, 0, 0), glm::vec3(1, 0, 0)});
            axis_vertices.push_back({o, glm::vec3(0, 1, 0)});
            axis_vertices.push_back({o + glm::vec3(0, l, 0), glm::vec3(0, 1, 0)});
            axis_vertices.push_back({o, glm::vec3(0, 0, 1)});
            axis_vertices.push_back({o + glm::vec3(0, 0, l), glm::vec3(0, 0, 1)});
        }
        if (ui_state.show_axes) {
            auto axes = scene.getJointAxisInfos(ui_state.show_revolute_only);
            for (const auto& a : axes) {
                if (!ui_state.show_non_revolute && !a.revolute) {
                    continue;
                }
                glm::vec3 c  = a.revolute ? glm::vec3(1.0f, 0.82f, 0.1f) : glm::vec3(0.45f, 0.8f, 1.0f);
                glm::vec3 p0 = a.world_origin - 0.5f * ui_state.axis_length * a.world_axis;
                glm::vec3 p1 = a.world_origin + 0.5f * ui_state.axis_length * a.world_axis;
                axis_vertices.push_back({p0, c});
                axis_vertices.push_back({p1, c});
            }
        }
        for (int i = 0; i < static_cast<int>(ik_state.marker_targets.size()); ++i) {
            if (!ik_state.marker_targets[i].initialized && !ensureMarkerTargetInitialized(i)) {
                continue;
            }
            const auto& target = ik_state.marker_targets[i];
            appendMarkerAxes(&axis_vertices, target.pos, target.rpy_deg, 0.12f, i == ik_state.selected_chain);
        }
        // Old custom marker is removed; ImGuizmo provides the interactive manipulator.

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            if (ik_state.selected_chain < 0 || ik_state.selected_chain >= static_cast<int>(ik_state.chains.size())) {
                ik_state.last_status = "IK失败：未选择链";
                return false;
            }
            const auto& chain_status = ik_state.chains[ik_state.selected_chain];
            if (ik_state.solve_mode == "single_chain" && !chain_status.ready) {
                ik_state.last_status = "IK失败：链未就绪";
                return false;
            }

            if (force_orientation_lock || ik_state.lock_orientation) {
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    ik_state.marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                    ik_state.marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                    ik_state.marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
                }
            }
            if (ik_state.solve_mode == "full_body" && fast_mode && prefer_position_only_target && !ik_state.lock_orientation) {
                // Realtime full-body drag uses position-priority target for robustness.
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    ik_state.marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                    ik_state.marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                    ik_state.marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
                }
            }
            saveActiveMarkerToTarget();

            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::mat4 active_target_world = markerWorldMatrix(marker_pos, marker_rpy_deg);

            if (ik_state.solve_mode == "full_body") {
                std::vector<glm::mat4> targets_world(static_cast<size_t>(ik_state.chains.size()), glm::mat4(1.0f));
                for (int i = 0; i < static_cast<int>(ik_state.chains.size()); ++i) {
                    if (i == ik_state.selected_chain) {
                        targets_world[static_cast<size_t>(i)] = active_target_world;
                        continue;
                    }

                    glm::vec3 tip_pos(0.0f);
                    glm::vec3 tip_rpy(0.0f);
                    if (ik_state.solver.fetchTipWorldPose(scene, i, &tip_pos, &tip_rpy)) {
                        const glm::vec3 tip_rpy_deg(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
                        targets_world[static_cast<size_t>(i)] = markerWorldMatrix(tip_pos, tip_rpy_deg);
                    } else if (ensureMarkerTargetInitialized(i)) {
                        const auto& target                    = ik_state.marker_targets[i];
                        targets_world[static_cast<size_t>(i)] = markerWorldMatrix(target.pos, target.rpy_deg);
                    } else {
                        targets_world[static_cast<size_t>(i)] = active_target_world;
                    }
                }

                IkSolveStats stats;
                const int solve_iters = fast_mode ? 1 : std::max(1, ik_state.full_body_iterations);
                if (ik_state.solver.solveFullBody(&scene, targets_world, solve_iters, ik_state.selected_chain, fast_mode,
                                                  prefer_position_only_target, &stats, &ik_state.last_status)) {
                    return true;
                }
                return false;
            }

            return ik_state.solver.solveSingleChain(&scene, ik_state.selected_chain, active_target_world, &ik_state.last_status);
        };

        // External RViz interactive marker target -> local marker -> IK.
        if (ik_state.use_external_target && ik_state.external_target_dirty && !ik_state.dragging_marker) {
            ik_state.marker_pos[0] = ik_state.external_target_pos.x;
            ik_state.marker_pos[1] = ik_state.external_target_pos.y;
            ik_state.marker_pos[2] = ik_state.external_target_pos.z;
            if (!ik_state.lock_orientation) {
                glm::vec3 rpy              = glm::eulerAngles(ik_state.external_target_quat);
                ik_state.marker_rpy_deg[0] = glm::degrees(rpy.x);
                ik_state.marker_rpy_deg[1] = glm::degrees(rpy.y);
                ik_state.marker_rpy_deg[2] = glm::degrees(rpy.z);
            }
            saveActiveMarkerToTarget();
            applyIkForActiveChain(false, false, ik_state.external_target_position_only);
            ik_state.external_target_dirty = false;
        }

        // RViz-like manipulator via ImGuizmo
        // Draw gizmo directly on the foreground drawlist of the 3D viewport area.
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetGizmoSizeClipSpace(ik_state.gizmo_size_clip_space);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(viewport_w), static_cast<float>(viewport_h));

        if (ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size())) {
            if (!ik_state.marker_initialized) {
                loadActiveMarkerFromTarget();
            }

            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            glm::mat4 gizmo_world  = markerWorldMatrix(marker_pos, marker_rpy_deg);
            ImGuizmo::OPERATION op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (ik_state.gizmo_operation == 0)
                op = ImGuizmo::TRANSLATE;
            if (ik_state.gizmo_operation == 1)
                op = ImGuizmo::ROTATE;
            ImGuizmo::MODE mode = ik_state.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;

            float snap_values[3] = {0.0f, 0.0f, 0.0f};
            float* snap_ptr      = nullptr;
            if (op == ImGuizmo::TRANSLATE && ik_state.translate_snap_enabled) {
                snap_values[0] = ik_state.translate_snap_step_m;
                snap_values[1] = ik_state.translate_snap_step_m;
                snap_values[2] = ik_state.translate_snap_step_m;
                snap_ptr       = snap_values;
            } else if (op == ImGuizmo::ROTATE && ik_state.rotate_snap_enabled) {
                snap_values[0] = ik_state.rotate_snap_step_deg;
                snap_values[1] = ik_state.rotate_snap_step_deg;
                snap_values[2] = ik_state.rotate_snap_step_deg;
                snap_ptr       = snap_values;
            }

            glm::mat4 gizmo_delta(1.0f);
            bool manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, mode, glm::value_ptr(gizmo_world),
                                                    glm::value_ptr(gizmo_delta), snap_ptr);
            bool gizmo_using = ImGuizmo::IsUsing();
            bool gizmo_over  = ImGuizmo::IsOver();
            bool current_drag_position_only = (op == ImGuizmo::TRANSLATE);
            if (ik_state.gizmo_operation == 2) {
                const glm::vec3 deltaTranslation(gizmo_delta[3]);
                const glm::quat deltaRotation = glm::quat_cast(gizmo_delta);
                const float rotationAmount    = glm::abs(glm::angle(deltaRotation));
                current_drag_position_only = glm::length(deltaTranslation) > 1e-6f && rotationAmount < glm::radians(0.25f);
            }
            if (manipulated || gizmo_using) {
                ik_state.dragging_marker       = true;
                ik_state.gizmo_drag_interacted = true;
                ik_state.gizmo_drag_position_only = current_drag_position_only;
                glm::vec3 p                    = glm::vec3(gizmo_world[3]);
                glm::quat q                    = glm::quat_cast(gizmo_world);
                glm::vec3 rpy                  = glm::eulerAngles(q);
                ik_state.marker_pos[0]         = p.x;
                ik_state.marker_pos[1]         = p.y;
                ik_state.marker_pos[2]         = p.z;
                if (!ik_state.lock_orientation) {
                    ik_state.marker_rpy_deg[0] = glm::degrees(rpy.x);
                    ik_state.marker_rpy_deg[1] = glm::degrees(rpy.y);
                    ik_state.marker_rpy_deg[2] = glm::degrees(rpy.z);
                }
                saveActiveMarkerToTarget();
                ik_state.gizmo_pose_dirty = true;
            } else {
                ik_state.dragging_marker = false;
            }

            // Realtime IK updates while dragging (RViz-like immediate feedback), throttled by frequency.
            if (gizmo_using && ik_state.gizmo_pose_dirty && ik_state.realtime_ik_during_drag) {
                float effective_hz = std::max(1.0f, ik_state.realtime_ik_hz);
                if (ik_state.solve_mode == "full_body") {
                    effective_hz = std::min(effective_hz, 12.0f);
                }
                const double interval_sec = 1.0 / static_cast<double>(effective_hz);
                if (ik_state.last_realtime_ik_apply_sec < 0.0 || (now_sec - ik_state.last_realtime_ik_apply_sec) >= interval_sec) {
                    applyIkForActiveChain(false, true, current_drag_position_only);
                    ik_state.last_realtime_ik_apply_sec = now_sec;
                    ik_state.gizmo_pose_dirty           = false;
                }
            }

            // Sync IK only when drag ends (mouse release / gizmo released)
            if (!gizmo_using && ik_state.gizmo_was_using && ik_state.gizmo_drag_interacted) {
                applyIkForActiveChain(false, false, ik_state.gizmo_drag_position_only);
                ik_state.gizmo_pose_dirty         = false;
                ik_state.gizmo_drag_interacted    = false;
                ik_state.gizmo_drag_position_only = true;
            }
            ik_state.gizmo_was_using = gizmo_using;
            ik_state.gizmo_was_over  = gizmo_over;
        } else {
            ik_state.gizmo_was_using = false;
            ik_state.gizmo_was_over  = false;
        }

        // Marker hover/pick in viewport (screen-space)
        if (false && ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size()) &&
            !ImGui::GetIO().WantCaptureMouse) {
            glm::vec3 m0(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            bool okc                = false;
            glm::vec2 sc            = worldToScreen(m0, view, proj, viewport_w, viewport_h, &okc);
            float dist_px           = okc ? glm::length(glm::vec2(static_cast<float>(x), static_cast<float>(y)) - sc) : 1e9f;
            bool in_view            = x <= viewport_w;
            bool left_now           = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            ik_state.marker_hovered = in_view && okc && dist_px < 36.0f;
            if (ik_state.marker_hovered) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            if (left_now && !ik_state.left_mouse_prev && ik_state.marker_hovered) {
                ik_state.dragging_marker = true;
                ik_state.drag_prev_x     = static_cast<float>(x);
                ik_state.drag_prev_y     = static_cast<float>(y);
                ik_state.drag_prev_valid = true;
            }
            if (!left_now) {
                ik_state.dragging_marker = false;
                ik_state.drag_prev_valid = false;
            }
            ik_state.left_mouse_prev = left_now;

            if (ik_state.dragging_marker) {
                glm::vec3 axis(1.0f, 0.0f, 0.0f);
                if (ik_state.drag_mode == 2 || ik_state.drag_mode == 5)
                    axis = glm::vec3(0.0f, 1.0f, 0.0f);
                if (ik_state.drag_mode == 3 || ik_state.drag_mode == 6)
                    axis = glm::vec3(0.0f, 0.0f, 1.0f);
                if (ik_state.drag_mode == 0) {
                    glm::vec3 eye_now = camera.eye();
                    glm::vec3 forward = glm::normalize(camera.target - eye_now);
                    glm::vec3 world_up(0.0f, 0.0f, 1.0f);
                    glm::vec3 right = glm::cross(forward, world_up);
                    if (glm::length(right) < 1e-6f)
                        right = glm::vec3(1.0f, 0.0f, 0.0f);
                    else
                        right = glm::normalize(right);
                    glm::vec3 up    = glm::normalize(glm::cross(right, forward));
                    float scale     = std::max(0.00008f, camera.distance * 0.00018f) * ik_state.translate_sensitivity;
                    glm::vec3 delta = right * static_cast<float>(dx) * scale - up * static_cast<float>(dy) * scale;
                    ik_state.marker_pos[0] += delta.x;
                    ik_state.marker_pos[1] += delta.y;
                    ik_state.marker_pos[2] += delta.z;
                } else {
                    bool oka = false, okb = false;
                    glm::vec2 sa = worldToScreen(m0, view, proj, viewport_w, viewport_h, &oka);
                    glm::vec2 sb = worldToScreen(m0 + axis * 0.16f, view, proj, viewport_w, viewport_h, &okb);
                    if (oka && okb) {
                        glm::vec2 axis_px = sb - sa;
                        float axis_px_len = glm::length(axis_px);
                        if (axis_px_len > 1e-4f) {
                            glm::vec2 mouse_delta(static_cast<float>(dx), static_cast<float>(dy));
                            float delta_along_px = glm::dot(mouse_delta, axis_px / axis_px_len);
                            if (ik_state.drag_mode >= 1 && ik_state.drag_mode <= 3) {
                                float scale = std::max(0.00008f, camera.distance * 0.00018f) * ik_state.translate_sensitivity;
                                float d     = delta_along_px * scale;
                                ik_state.marker_pos[0] += axis.x * d;
                                ik_state.marker_pos[1] += axis.y * d;
                                ik_state.marker_pos[2] += axis.z * d;
                            } else {
                                int ridx = ik_state.drag_mode - 4;
                                if (ridx >= 0 && ridx < 3) {
                                    glm::vec2 center = sa;
                                    glm::vec2 v0(ik_state.drag_prev_x - center.x, ik_state.drag_prev_y - center.y);
                                    glm::vec2 v1(static_cast<float>(x) - center.x, static_cast<float>(y) - center.y);
                                    float l0 = glm::length(v0);
                                    float l1 = glm::length(v1);
                                    if (ik_state.drag_prev_valid && l0 > 1.0f && l1 > 1.0f) {
                                        v0 /= l0;
                                        v1 /= l1;
                                        float c   = glm::clamp(glm::dot(v0, v1), -1.0f, 1.0f);
                                        float ang = std::acos(c);
                                        float sgn = (v0.x * v1.y - v0.y * v1.x) >= 0.0f ? 1.0f : -1.0f;
                                        float deg = glm::degrees(ang) * sgn * ik_state.rotate_sensitivity;
                                        ik_state.marker_rpy_deg[ridx] += deg;
                                    }
                                }
                            }
                        }
                    }
                }
                ik_state.drag_prev_x     = static_cast<float>(x);
                ik_state.drag_prev_y     = static_cast<float>(y);
                ik_state.drag_prev_valid = true;
                saveActiveMarkerToTarget();
                applyIkForActiveChain(false, false, false);
            }
        }
        line_renderer.draw(line_shader, axis_vertices, view, proj, ui_state.axis_line_width);

        glViewport(viewport_w, 0, panel_w, fb_h);
        glDisable(GL_DEPTH_TEST);
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(viewport_w), 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(panel_w), static_cast<float>(fb_h)), ImGuiCond_Always);
        ImGui::Begin("机器人运动学调试", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

        {
            ImGuiIO& io            = ImGui::GetIO();
            const float grip_width = 8.0f;
            ImVec2 window_pos      = ImGui::GetWindowPos();
            ImVec2 window_size     = ImGui::GetWindowSize();
            ImVec2 grip_min(window_pos.x, window_pos.y);
            ImVec2 grip_max(window_pos.x + grip_width, window_pos.y + window_size.y);
            ImGui::SetCursorScreenPos(grip_min);
            ImGui::InvisibleButton("##sidebar_resize_grip_kin", ImVec2(grip_width, window_size.y));
            bool grip_hovered            = ImGui::IsItemHovered();
            bool grip_active             = ImGui::IsItemActive();
            ui_state.panel_resize_active = grip_active;
            if (grip_hovered || grip_active) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            }
            if (grip_active) {
                ui_state.panel_width = std::clamp(ui_state.panel_width - io.MouseDelta.x, panel_min, panel_max);
            }
            ImU32 grip_color =
                grip_active ? IM_COL32(120, 200, 255, 220) : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
            ImGui::GetWindowDrawList()->AddRectFilled(grip_min, grip_max, grip_color, 2.0f);
            ImGui::SetCursorPosY(8.0f);
        }

        ImGui::TextWrapped("URDF: %s", urdf_path.c_str());
        ImGui::TextDisabled("视角：左键旋转，中键/Shift+左键平移，右键拖动缩放，滚轮缩放");
        ImGui::Separator();
        ImGui::Checkbox("显示关节轴", &ui_state.show_axes);
        ImGui::Checkbox("仅旋转关节轴", &ui_state.show_revolute_only);
        ImGui::Checkbox("显示非旋转关节", &ui_state.show_non_revolute);
        ImGui::Checkbox("显示世界坐标轴", &ui_state.show_world_axes);
        ImGui::Checkbox("固定底座模式", &ui_state.lock_base);
        ImGui::SliderFloat("关节轴长度", &ui_state.axis_length, 0.03f, 0.5f, "%.3f");
        ImGui::SliderFloat("线宽", &ui_state.axis_line_width, 1.0f, 6.0f, "%.1f");
        ImGui::SliderFloat("世界轴长度", &ui_state.world_axis_length, 0.1f, 1.5f, "%.2f");
        ImGui::SliderFloat("地面网格尺寸", &ui_state.grid_size, 1.0f, 20.0f, "%.1f");
        ImGui::SliderInt("地面网格密度", &ui_state.grid_count, 10, 120);
        ImGui::Separator();

        ImGui::InputText("关节过滤", ui_state.joint_filter, sizeof(ui_state.joint_filter));
        std::string filter = ui_state.joint_filter;
        std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        auto joints          = scene.getJointInfos();
        int revolute_count   = 0;
        int clamped_count    = 0;
        float min_margin_deg = 1e9f;
        std::string min_margin_joint;
        for (const auto& j : joints) {
            if (j.revolute)
                revolute_count++;
            if (j.position < j.min_angle - 1e-5f || j.position > j.max_angle + 1e-5f)
                clamped_count++;
            if (j.revolute) {
                float d0 = std::fabs(j.position - j.min_angle);
                float d1 = std::fabs(j.max_angle - j.position);
                float m  = glm::degrees(std::min(d0, d1));
                if (m < min_margin_deg) {
                    min_margin_deg   = m;
                    min_margin_joint = j.name;
                }
            }
        }
        ImGui::Text("关节总数: %d  旋转关节: %d  越界关节: %d", static_cast<int>(joints.size()), revolute_count, clamped_count);
        if (!min_margin_joint.empty()) {
            ImVec4 c = (min_margin_deg < 3.0f)
                           ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
                           : ((min_margin_deg < 8.0f) ? ImVec4(1.0f, 0.75f, 0.25f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f));
            ImGui::TextColored(c, "最小限位裕量: %.2f deg (%s)", min_margin_deg, min_margin_joint.c_str());
        }

        if (ImGui::Button("旋转关节全部归零")) {
            for (const auto& j : joints) {
                if (j.revolute) {
                    scene.setJointPositionByName(j.name, 0.0f);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("一键夹紧到限位内")) {
            for (const auto& j : joints) {
                if (!j.revolute) {
                    continue;
                }
                float v = std::clamp(j.position, j.min_angle, j.max_angle);
                scene.setJointPositionByName(j.name, v);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("保存当前姿态")) {
            ui_state.pose_snapshot.clear();
            for (const auto& j : joints) {
                ui_state.pose_snapshot[j.name] = j.position;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("恢复保存姿态") && !ui_state.pose_snapshot.empty()) {
            for (const auto& [name, value] : ui_state.pose_snapshot) {
                scene.setJointPositionByName(name, value);
            }
        }

        ImGui::Separator();
        ImGui::TextUnformatted("末端 Marker IK（TRAC-IK）");
        if (!ik_state.chains.empty()) {
            std::vector<const char*> chain_labels;
            chain_labels.reserve(ik_state.chains.size());
            for (const auto& c : ik_state.chains) {
                chain_labels.push_back(c.config.label.c_str());
            }
            if (ImGui::Combo("控制链", &ik_state.selected_chain, chain_labels.data(), static_cast<int>(chain_labels.size()))) {
                ik_state.marker_initialized = false;
                loadActiveMarkerFromTarget();
            }
            int solve_mode_index           = (ik_state.solve_mode == "full_body") ? 1 : 0;
            const char* solve_mode_items[] = {"single_chain（单链）", "full_body（全身）"};
            if (ImGui::Combo("求解模式", &solve_mode_index, solve_mode_items, IM_ARRAYSIZE(solve_mode_items))) {
                ik_state.solve_mode  = (solve_mode_index == 1) ? "full_body" : "single_chain";
                ik_state.last_status = "已切换IK求解模式";
            }
            if (ik_state.solve_mode == "full_body") {
                int backend_index           = (ik_state.full_body_backend == "wbc_chain_ik") ? 1 : 0;
                const char* backend_items[] = {"flex_ik", "wbc_chain_ik"};
                if (ImGui::Combo("全身IK后端", &backend_index, backend_items, IM_ARRAYSIZE(backend_items))) {
                    ik_state.full_body_backend = (backend_index == 1) ? "wbc_chain_ik" : "flex_ik";
                    ik_state.solver.setFullBodyBackend(ik_state.full_body_backend);
                    ik_state.last_status = std::string("已切换全身IK后端: ") + ik_state.full_body_backend;
                }
                ImGui::DragInt("全身迭代次数", &ik_state.full_body_iterations, 1.0f, 1, 30);
                ik_state.full_body_iterations = std::max(1, ik_state.full_body_iterations);
                ImGui::TextDisabled("彩色目标轴: 亮色=当前控制链，淡色=其他链");
            }
            ImGui::Checkbox("使用RViz目标位姿", &ik_state.use_external_target);
            ImGui::TextDisabled("外部位姿topic: %s", ik_state.external_target_topic.c_str());
            if (!ik_state.external_target_expected_frame.empty()) {
                ImGui::TextDisabled("外部位姿frame过滤: %s", ik_state.external_target_expected_frame.c_str());
            } else {
                ImGui::TextDisabled("外部位姿frame过滤: <未设置>");
            }
            if (ik_state.external_target_received) {
                double now_ros_sec = ros::Time::now().toSec();
                double age_sec     = std::max(0.0, now_ros_sec - ik_state.external_target_last_recv_sec);
                ImGui::Text("外部位姿: 已接收, frame=%s, %.2fs前",
                            ik_state.external_target_last_frame.empty() ? "<empty>" : ik_state.external_target_last_frame.c_str(), age_sec);
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 1.0f), "外部位姿: 未接收到消息");
            }

            const auto& chain_status = ik_state.chains[ik_state.selected_chain];
            if (ik_state.solve_mode == "single_chain" && !chain_status.ready) {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "IK链不可用: %s", chain_status.error.c_str());
            } else {
                if (!ik_state.marker_initialized) {
                    loadActiveMarkerFromTarget();
                }
                ImGui::Checkbox("锁定末端姿态", &ik_state.lock_orientation);
                ImGui::TextUnformatted("Gizmo 模式");
                ImGui::RadioButton("平移", &ik_state.gizmo_operation, 0);
                ImGui::SameLine();
                ImGui::RadioButton("旋转", &ik_state.gizmo_operation, 1);
                ImGui::SameLine();
                ImGui::RadioButton("平移+旋转", &ik_state.gizmo_operation, 2);
                ImGui::TextUnformatted("坐标系");
                if (ImGui::RadioButton("WORLD", ik_state.gizmo_world_mode)) {
                    ik_state.gizmo_world_mode = true;
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("LOCAL", !ik_state.gizmo_world_mode)) {
                    ik_state.gizmo_world_mode = false;
                }
                ImGui::SliderFloat("Gizmo 尺寸", &ik_state.gizmo_size_clip_space, 0.10f, 0.40f, "%.2f");
                ImGui::Checkbox("拖动时实时IK", &ik_state.realtime_ik_during_drag);
                if (ik_state.realtime_ik_during_drag) {
                    ImGui::SliderFloat("实时IK频率(Hz)", &ik_state.realtime_ik_hz, 5.0f, 120.0f, "%.0f");
                    if (ik_state.solve_mode == "full_body") {
                        ImGui::TextDisabled("full_body 拖动时频率自动上限 12Hz（防卡顿）");
                        ImGui::TextDisabled("full_body 平移拖动走位置优先，旋转拖动走姿态求解");
                    }
                }
                ImGui::Checkbox("平移吸附", &ik_state.translate_snap_enabled);
                if (ik_state.translate_snap_enabled) {
                    ImGui::DragFloat("平移步长(m)", &ik_state.translate_snap_step_m, 0.001f, 0.001f, 0.20f, "%.3f");
                }
                ImGui::Checkbox("旋转吸附", &ik_state.rotate_snap_enabled);
                if (ik_state.rotate_snap_enabled) {
                    ImGui::DragFloat("旋转步长(度)", &ik_state.rotate_snap_step_deg, 0.2f, 0.2f, 45.0f, "%.1f");
                }
                ImGui::TextDisabled("直接在3D视窗抓取 Gizmo 轴/圆环进行平移或旋转");
                bool marker_edited = false;
                marker_edited |= ImGui::DragFloat3("Marker 位置(m)", ik_state.marker_pos, 0.002f, -2.0f, 2.0f, "%.4f");
                ImGui::BeginDisabled(ik_state.lock_orientation);
                marker_edited |= ImGui::DragFloat3("Marker 姿态RPY(度)", ik_state.marker_rpy_deg, 0.2f, -180.0f, 180.0f, "%.2f");
                ImGui::EndDisabled();
                if (marker_edited) {
                    saveActiveMarkerToTarget();
                }
                if (ImGui::Button("从当前末端同步Marker")) {
                    glm::vec3 tip_pos(0.0f);
                    glm::vec3 tip_rpy(0.0f);
                    if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                        ik_state.marker_pos[0]      = tip_pos.x;
                        ik_state.marker_pos[1]      = tip_pos.y;
                        ik_state.marker_pos[2]      = tip_pos.z;
                        ik_state.marker_rpy_deg[0]  = glm::degrees(tip_rpy.x);
                        ik_state.marker_rpy_deg[1]  = glm::degrees(tip_rpy.y);
                        ik_state.marker_rpy_deg[2]  = glm::degrees(tip_rpy.z);
                        ik_state.marker_initialized = true;
                        saveActiveMarkerToTarget();
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("求解 IK 并应用")) {
                    applyIkForActiveChain(false, false, false);
                }
                if (!ik_state.last_status.empty()) {
                    ImGui::TextUnformatted(ik_state.last_status.c_str());
                }
                ImGui::TextDisabled("base=%s  tip=%s", chain_status.config.base_link.c_str(), chain_status.config.tip_link.c_str());

                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    glm::vec3 marker_p(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
                    float pos_err_mm = glm::length(marker_p - tip_pos) * 1000.0f;
                    glm::vec3 tip_rpy_deg(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
                    glm::vec3 marker_rpy(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
                    glm::vec3 drpy = glm::abs(marker_rpy - tip_rpy_deg);
                    ImVec4 ce      = (pos_err_mm < 2.0f)
                                         ? ImVec4(0.6f, 0.95f, 0.6f, 1.0f)
                                         : ((pos_err_mm < 8.0f) ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f) : ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                    ImGui::TextColored(ce, "末端误差: 位置 %.2f mm, 姿态 %.2f/%.2f/%.2f deg", pos_err_mm, drpy.x, drpy.y, drpy.z);
                }
            }
        }
        ImGui::Separator();
        ImGui::TextUnformatted("姿态关键帧回放");
        if (ImGui::Button("记录关键帧")) {
            PoseKeyframe kf;
            kf.t = playback_state.keyframes.empty() ? 0.0 : (playback_state.keyframes.back().t + 1.0);
            for (const auto& j : joints) {
                if (j.revolute) {
                    kf.joints[j.name] = j.position;
                }
            }
            playback_state.keyframes.push_back(std::move(kf));
            playback_state.play_time = static_cast<float>(playback_state.keyframes.back().t);
        }
        ImGui::SameLine();
        if (ImGui::Button(playback_state.playing ? "暂停回放" : "开始回放")) {
            if (playback_state.keyframes.size() >= 2) {
                playback_state.playing = !playback_state.playing;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("清空关键帧")) {
            playback_state.keyframes.clear();
            playback_state.playing   = false;
            playback_state.play_time = 0.0f;
        }
        ImGui::Checkbox("循环回放", &playback_state.loop);
        ImGui::SliderFloat("回放倍速", &playback_state.play_speed, 0.1f, 3.0f, "%.2fx");
        if (!playback_state.keyframes.empty()) {
            float total = static_cast<float>(playback_state.keyframes.back().t);
            ImGui::SliderFloat("回放时间", &playback_state.play_time, 0.0f, std::max(0.0f, total), "%.2f s");
            ImGui::Text("关键帧数: %d", static_cast<int>(playback_state.keyframes.size()));
        } else {
            ImGui::TextDisabled("暂无关键帧，先点击“记录关键帧”。");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("关节调试");
        ui_state.joint_section_height =
            std::clamp(ui_state.joint_section_height, 120.0f, std::max(140.0f, ImGui::GetContentRegionAvail().y - 160.0f));
        if (ImGui::BeginTable("joint_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0.0f, ui_state.joint_section_height))) {
            ImGui::TableSetupColumn("关节");
            ImGui::TableSetupColumn("角度滑条");
            ImGui::TableSetupColumn("限位");
            ImGui::TableSetupColumn("数值输入(度)");
            ImGui::TableHeadersRow();

            for (const auto& j : joints) {
                if (!ui_state.show_non_revolute && !j.revolute) {
                    continue;
                }
                std::string name_lower = j.name;
                std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!filter.empty() && name_lower.find(filter) == std::string::npos) {
                    continue;
                }

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(j.name.c_str());
                ImGui::TableSetColumnIndex(1);
                float value = j.position;
                if (j.revolute) {
                    ImGui::PushItemWidth(-1);
                    std::string slider_id = "##slider_" + j.name;
                    if (ImGui::SliderAngle(slider_id.c_str(), &value, glm::degrees(j.min_angle), glm::degrees(j.max_angle))) {
                        scene.setJointPositionByName(j.name, value);
                    }
                    ImGui::PopItemWidth();
                } else {
                    ImGui::TextDisabled("不适用");
                }

                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f / %.2f deg", glm::degrees(j.min_angle), glm::degrees(j.max_angle));
                ImGui::TableSetColumnIndex(3);
                float degree_val     = glm::degrees(j.position);
                std::string input_id = "##deg_" + j.name;
                if (ImGui::InputFloat(input_id.c_str(), &degree_val, 0.1f, 1.0f, "%.2f")) {
                    float rad = glm::radians(degree_val);
                    scene.setJointPositionByName(j.name, rad);
                }
            }
            ImGui::EndTable();
        }

        {
            ImGuiIO& io   = ImGui::GetIO();
            float avail_w = ImGui::GetContentRegionAvail().x;
            ImGui::InvisibleButton("##joint_tf_splitter", ImVec2(avail_w, 8.0f));
            bool hovered = ImGui::IsItemHovered();
            bool active  = ImGui::IsItemActive();
            if (hovered || active) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            }
            if (active) {
                ui_state.joint_section_height += io.MouseDelta.y;
            }
            ImVec2 min = ImGui::GetItemRectMin();
            ImVec2 max = ImGui::GetItemRectMax();
            ImU32 c    = active ? IM_COL32(120, 200, 255, 220) : (hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
            ImGui::GetWindowDrawList()->AddRectFilled(min, max, c, 2.0f);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("TF 视图");
        ImGui::InputText("TF过滤", ui_state.tf_filter, sizeof(ui_state.tf_filter));
        std::string tf_filter = ui_state.tf_filter;
        std::transform(tf_filter.begin(), tf_filter.end(), tf_filter.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        auto tfs = scene.getLinkTfInfos();
        if (ImGui::BeginTable("tf_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
            ImGui::TableSetupColumn("Link");
            ImGui::TableSetupColumn("父Link");
            ImGui::TableSetupColumn("位置 xyz(m)");
            ImGui::TableSetupColumn("姿态 rpy(deg)");
            ImGui::TableHeadersRow();
            for (const auto& tf : tfs) {
                std::string key       = tf.name + " " + tf.parent_name;
                std::string key_lower = key;
                std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!tf_filter.empty() && key_lower.find(tf_filter) == std::string::npos) {
                    continue;
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(tf.name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(tf.parent_name.empty() ? "-" : tf.parent_name.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3f, %.3f, %.3f", tf.world_position.x, tf.world_position.y, tf.world_position.z);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.1f, %.1f, %.1f", glm::degrees(tf.world_rpy.x), glm::degrees(tf.world_rpy.y), glm::degrees(tf.world_rpy.z));
            }
            ImGui::EndTable();
        }

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    glDeleteProgram(mesh_shader);
    glDeleteProgram(line_shader);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    if (ros::isStarted()) {
        ros::shutdown();
    }
    return 0;
}
