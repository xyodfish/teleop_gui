#include "teleop_viewer/app.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace omnilink::teleop_viewer {
namespace {

const char* kVertexShader = R"(
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

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoords = aTexCoords;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

const char* kFragmentShader = R"(
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

void main()
{
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * diffuseColor;

    vec3 ambient = 0.5 * diffuseColor;

    vec3 result = ambient + diffuse;
    if (hasTexture) {
        result *= texture(texture_diffuse1, TexCoords).rgb;
    }
    color = vec4(result, 1.0);
}
)";

const char* kWaveMetricLabels[] = {"位置(rad)", "速度", "力矩", "电流"};

const char* WaveMetricKeySuffix(int metric_index) {
    switch (metric_index) {
        case 0:
            return "position";
        case 1:
            return "velocity";
        case 2:
            return "effort";
        case 3:
            return "current";
        default:
            return "position";
    }
}

void DrawWaveformWithAxes(const char* id, const std::vector<float>& values, float min_value, float max_value, ImVec2 size) {
    float width = size.x > 0.0f ? size.x : ImGui::GetContentRegionAvail().x;
    float height = size.y > 0.0f ? size.y : 180.0f;
    if (width < 120.0f) width = 120.0f;
    if (height < 90.0f) height = 90.0f;

    const float margin_left = 56.0f;
    const float margin_right = 10.0f;
    const float margin_top = 10.0f;
    const float margin_bottom = 24.0f;

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id, ImVec2(width, height));
    ImVec2 canvas_min = ImGui::GetItemRectMin();
    ImVec2 canvas_max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(canvas_min, canvas_max, IM_COL32(22, 24, 28, 255), 4.0f);
    dl->AddRect(canvas_min, canvas_max, IM_COL32(80, 80, 90, 255), 4.0f);

    ImVec2 plot_min(canvas_pos.x + margin_left, canvas_pos.y + margin_top);
    ImVec2 plot_max(canvas_pos.x + width - margin_right, canvas_pos.y + height - margin_bottom);
    if (plot_max.x <= plot_min.x || plot_max.y <= plot_min.y) {
        return;
    }

    dl->AddRect(plot_min, plot_max, IM_COL32(120, 120, 130, 255), 0.0f);

    const int y_ticks = 5;
    for (int i = 0; i < y_ticks; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(y_ticks - 1);
        float y = plot_min.y + t * (plot_max.y - plot_min.y);
        dl->AddLine(ImVec2(plot_min.x, y), ImVec2(plot_max.x, y), IM_COL32(60, 60, 70, 255), 1.0f);

        float v = max_value - t * (max_value - min_value);
        char label[32];
        std::snprintf(label, sizeof(label), "%.3f", v);
        dl->AddText(ImVec2(canvas_pos.x + 4.0f, y - 7.0f), IM_COL32(190, 190, 200, 255), label);
    }

    if (!values.empty()) {
        int n = static_cast<int>(values.size());
        auto x_to_screen = [&](int idx) {
            if (n <= 1) return plot_min.x;
            float x_t = static_cast<float>(idx) / static_cast<float>(n - 1);
            return plot_min.x + x_t * (plot_max.x - plot_min.x);
        };
        auto y_to_screen = [&](float v) {
            float y_t = (v - min_value) / (max_value - min_value);
            y_t = std::clamp(y_t, 0.0f, 1.0f);
            return plot_max.y - y_t * (plot_max.y - plot_min.y);
        };

        for (int i = 1; i < n; ++i) {
            dl->AddLine(ImVec2(x_to_screen(i - 1), y_to_screen(values[i - 1])),
                        ImVec2(x_to_screen(i), y_to_screen(values[i])),
                        IM_COL32(102, 204, 255, 255), 1.8f);
        }

        int mid = n > 1 ? (n - 1) / 2 : 0;
        int x_ticks[3] = {0, mid, n - 1};
        for (int i = 0; i < 3; ++i) {
            float x = x_to_screen(x_ticks[i]);
            dl->AddLine(ImVec2(x, plot_max.y), ImVec2(x, plot_max.y + 4.0f), IM_COL32(170, 170, 180, 255), 1.0f);

            char label[24];
            std::snprintf(label, sizeof(label), "%d", x_ticks[i]);
            ImVec2 text_size = ImGui::CalcTextSize(label);
            dl->AddText(ImVec2(x - text_size.x * 0.5f, plot_max.y + 6.0f), IM_COL32(190, 190, 200, 255), label);
        }
    }
}

}  // namespace

RobotViewerApp::RobotViewerApp(ViewerConfig config) : config_(std::move(config)) {
    camera_.distance = config_.camera.distance;
    camera_.yaw      = config_.camera.yaw;
    camera_.pitch    = config_.camera.pitch;
    camera_.target   = config_.camera.target;

    camera_.rotate_speed = config_.camera.rotate_speed;
    camera_.zoom_scale   = config_.camera.zoom_scale;
    camera_.dolly_scale  = config_.camera.dolly_scale;
    camera_.pan_scale    = config_.camera.pan_scale;
    camera_.min_distance = config_.camera.min_distance;
    camera_.max_distance = config_.camera.max_distance;

    use_sensor_to_drive_robot_   = true;
    only_show_master_arm_groups_ = config_.ui.only_show_master_arm_groups;
    fix_base_like_mujoco_        = config_.ui.fix_base_like_mujoco;

    side_panel_width_        = config_.ui.side_panel_width;
    collapsed_sidebar_width_ = config_.ui.collapsed_sidebar_width;
    sidebar_collapsed_       = config_.ui.sidebar_collapsed_default;
    waveform_history_size_   = std::max(50, config_.ui.waveform_history_size);
    waveform_plot_height_    = std::max(80.0f, config_.ui.waveform_plot_height);
    baseline_warn_deg_       = std::max(0.1f, config_.ui.baseline_warn_delta_deg);
    show_only_active_alarms_ = config_.ui.alarm_show_only_active_default;
    alarm_trigger_frames_    = std::max(1, config_.ui.alarm_trigger_frames);
    record_output_dir_       = config_.ui.record_output_dir.empty() ? "logs" : config_.ui.record_output_dir;
    app_start_time_          = std::chrono::steady_clock::now();
}

RobotViewerApp::~RobotViewerApp() { shutdown(); }

void RobotViewerApp::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    (void)xoffset;
    auto* app = static_cast<RobotViewerApp*>(glfwGetWindowUserPointer(window));
    if (app) {
        app->onScroll(yoffset);
    }
}

void RobotViewerApp::onScroll(double yoffset) { scroll_offset_ += yoffset; }

bool RobotViewerApp::initWindow() {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(config_.window.width, config_.window.height, config_.window.title.c_str(), nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetScrollCallback(window_, ScrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to init GLAD" << std::endl;
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    return true;
}

bool RobotViewerApp::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    const ImWchar* glyph_ranges = io.Fonts->GetGlyphRangesChineseFull();
    float font_size = std::max(12.0f, config_.ui.cjk_font_size);
    bool font_loaded = false;
    std::string loaded_font_path;

    auto try_load_font = [&](const std::string& path) -> bool {
        if (path.empty() || !std::filesystem::exists(path)) {
            return false;
        }
        ImFontConfig font_cfg;
        font_cfg.OversampleH = 2;
        font_cfg.OversampleV = 1;
        font_cfg.PixelSnapH  = true;
        ImFont* font = io.Fonts->AddFontFromFileTTF(path.c_str(), font_size, &font_cfg, glyph_ranges);
        if (!font) {
            return false;
        }
        loaded_font_path = path;
        return true;
    };

    if (!config_.ui.cjk_font_path.empty()) {
        font_loaded = try_load_font(config_.ui.cjk_font_path);
        if (!font_loaded) {
            std::cerr << "[robot_viewer] Failed to load cjk_font_path: " << config_.ui.cjk_font_path << std::endl;
        }
    }

    if (!font_loaded) {
        const std::vector<std::string> candidates = {
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"};

        for (const auto& path : candidates) {
            if (try_load_font(path)) {
                font_loaded = true;
                break;
            }
        }
    }

    if (!font_loaded) {
        io.Fonts->AddFontDefault();
        std::cerr << "[robot_viewer] No CJK font found. Chinese text may show as '?'. "
                  << "Please set ui.cjk_font_path in config/robot_viewer.yaml." << std::endl;
    } else {
        std::cout << "[robot_viewer] Loaded CJK font: " << loaded_font_path << " (size=" << font_size << ")"
                  << std::endl;
    }

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    return true;
}

bool RobotViewerApp::initShader() {
    shader_ = glCreateProgram();

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &kVertexShader, nullptr);
    glCompileShader(vs);

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &kFragmentShader, nullptr);
    glCompileShader(fs);

    glAttachShader(shader_, vs);
    glAttachShader(shader_, fs);
    glLinkProgram(shader_);

    glDeleteShader(vs);
    glDeleteShader(fs);
    return true;
}

bool RobotViewerApp::initScene() {
    if (!scene_.loadURDF(config_.robot.urdf_path)) {
        return false;
    }

    sensor_ready_ = sensor_subscriber_.start(config_.sensor.topic, config_.sensor.node_name);
    if (!sensor_ready_) {
        use_sensor_to_drive_robot_ = false;
    }

    scene_.setFixedBaseMode(fix_base_like_mujoco_);
    if (config_.ui.auto_start_recording) {
        startRecording(nowSec());
    }
    return true;
}

void RobotViewerApp::shutdown() {
    stopRecording();
    sensor_subscriber_.stop();

    if (shader_) {
        glDeleteProgram(shader_);
        shader_ = 0;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void RobotViewerApp::processCameraInput(bool imgui_want_capture_mouse) {
    if (imgui_want_capture_mouse) {
        scroll_offset_ = 0.0;
        dragging_      = false;
        drag_mode_     = CameraDragMode::None;
        return;
    }

    if (std::abs(scroll_offset_) > 1e-6) {
        camera_.zoom(static_cast<float>(scroll_offset_));
        scroll_offset_ = 0.0;
    }

    bool lmb = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    bool mmb = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    bool rmb = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    bool shift_pressed = glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                         glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    CameraDragMode desired_mode = CameraDragMode::None;
    if (mmb || (lmb && shift_pressed)) {
        desired_mode = CameraDragMode::Pan;
    } else if (rmb) {
        desired_mode = CameraDragMode::Dolly;
    } else if (lmb) {
        desired_mode = CameraDragMode::Rotate;
    }

    if (desired_mode == CameraDragMode::None) {
        dragging_ = false;
        drag_mode_ = CameraDragMode::None;
        return;
    }

    double xpos = 0.0;
    double ypos = 0.0;
    glfwGetCursorPos(window_, &xpos, &ypos);

    if (!dragging_ || drag_mode_ != desired_mode) {
        dragging_ = true;
        drag_mode_ = desired_mode;
        last_x_ = xpos;
        last_y_ = ypos;
        return;
    }

    double dx = xpos - last_x_;
    double dy = ypos - last_y_;
    last_x_ = xpos;
    last_y_ = ypos;

    if (drag_mode_ == CameraDragMode::Rotate) {
        camera_.rotate(static_cast<float>(dx), static_cast<float>(dy));
    } else if (drag_mode_ == CameraDragMode::Pan) {
        camera_.pan(static_cast<float>(dx), static_cast<float>(dy));
    } else if (drag_mode_ == CameraDragMode::Dolly) {
        camera_.dolly(static_cast<float>(-dy));
    }
}

double RobotViewerApp::nowSec() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - app_start_time_).count();
}

std::string RobotViewerApp::composeJointKey(const SensorJointSample& sample) const { return sample.group + "/" + sample.name; }

RobotViewerApp::JointDiagState RobotViewerApp::evaluateJoint(const SensorJointSample& sample) const {
    JointDiagState diag;

    RobotScene::JointInfo info;
    diag.has_info = scene_.getJointInfo(sample.name, &info);
    if (diag.has_info) {
        diag.min_angle = info.min_angle;
        diag.max_angle = info.max_angle;
    }

    diag.invalid = !sample.has_position || !std::isfinite(sample.position);
    diag.no_match = !diag.has_info;
    if (diag.has_info && !diag.invalid) {
        diag.out_of_range = sample.position < diag.min_angle - config_.ui.out_of_range_margin ||
                            sample.position > diag.max_angle + config_.ui.out_of_range_margin;
    }
    return diag;
}

void RobotViewerApp::updateAlarmForJoint(const SensorJointSample& sample, const JointDiagState& diag, double now_sec) {
    const std::string key = composeJointKey(sample);
    bool bad = diag.invalid || diag.no_match || diag.out_of_range;

    int& streak = bad_frame_streak_[key];
    if (bad) {
        streak++;
    } else {
        streak = 0;
        auto it = alarms_.find(key);
        if (it != alarms_.end()) {
            it->second.active = false;
            it->second.last_seen_s = now_sec;
        }
        return;
    }

    if (streak < alarm_trigger_frames_) {
        return;
    }

    std::string reason = "OUT_OF_RANGE";
    if (diag.invalid) {
        reason = "INVALID";
    } else if (diag.no_match) {
        reason = "NO_URDF_MATCH";
    }

    auto it = alarms_.find(key);
    if (it == alarms_.end()) {
        AlarmEntry alarm;
        alarm.group = sample.group;
        alarm.joint = sample.name;
        alarm.reason = reason;
        alarm.trigger_count = 1;
        alarm.first_seen_s = now_sec;
        alarm.last_seen_s = now_sec;
        alarm.active = true;
        alarm.acknowledged = false;
        alarms_[key] = std::move(alarm);
        return;
    }

    AlarmEntry& alarm = it->second;
    if (!alarm.active) {
        alarm.first_seen_s = now_sec;
        alarm.trigger_count = 0;
        alarm.acknowledged = false;
    }
    alarm.group = sample.group;
    alarm.joint = sample.name;
    alarm.reason = reason;
    alarm.active = true;
    alarm.last_seen_s = now_sec;
    alarm.trigger_count++;
}

void RobotViewerApp::updateInputRate(uint64_t msg_count, double now_sec) {
    if (msg_count > last_msg_count_seen_) {
        uint64_t delta = msg_count - last_msg_count_seen_;
        for (uint64_t i = 0; i < delta; ++i) {
            msg_arrival_times_sec_.push_back(now_sec);
        }
        const double window_sec = 2.0;
        while (!msg_arrival_times_sec_.empty() && now_sec - msg_arrival_times_sec_.front() > window_sec) {
            msg_arrival_times_sec_.pop_front();
        }
    }
    last_msg_count_seen_ = msg_count;

    if (msg_arrival_times_sec_.size() >= 2) {
        double span = msg_arrival_times_sec_.back() - msg_arrival_times_sec_.front();
        if (span > 1e-6) {
            input_rate_hz_ = static_cast<double>(msg_arrival_times_sec_.size() - 1) / span;
        }
    } else {
        input_rate_hz_ = 0.0;
    }
}

void RobotViewerApp::startRecording(double now_sec) {
    if (recording_) {
        return;
    }

    record_error_.clear();

    std::error_code ec;
    std::filesystem::create_directories(record_output_dir_, ec);
    if (ec) {
        record_error_ = "Create record dir failed: " + ec.message();
        return;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm_buf;
    std::tm* tm_ptr = localtime_r(&t, &tm_buf);
    if (!tm_ptr) {
        record_error_ = "localtime failed";
        return;
    }

    std::ostringstream path_ss;
    path_ss << record_output_dir_ << "/teleop_session_" << std::put_time(tm_ptr, "%Y%m%d_%H%M%S") << ".csv";
    record_file_path_ = path_ss.str();

    record_file_.open(record_file_path_, std::ios::out | std::ios::trunc);
    if (!record_file_.is_open()) {
        record_error_ = "Open record file failed: " + record_file_path_;
        return;
    }

    record_file_ << "time_s,group,joint,position,velocity,effort,current,invalid,no_urdf_match,out_of_range,health\n";
    recorded_rows_ = 0;
    recording_ = true;
    (void)now_sec;
}

void RobotViewerApp::stopRecording() {
    if (record_file_.is_open()) {
        record_file_.flush();
        record_file_.close();
    }
    recording_ = false;
}

void RobotViewerApp::recordCurrentSamples(double now_sec, const std::unordered_map<std::string, JointDiagState>& diag_map) {
    if (!recording_ || !record_file_.is_open()) {
        return;
    }

    auto write_value = [this](bool has_value, double value) {
        if (has_value && std::isfinite(value)) {
            record_file_ << value;
        }
    };

    for (const auto& sample : latest_sensor_samples_) {
        if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
            continue;
        }

        const std::string key = composeJointKey(sample);
        JointDiagState diag;
        auto it = diag_map.find(key);
        if (it != diag_map.end()) {
            diag = it->second;
        } else {
            diag = evaluateJoint(sample);
        }

        const char* health = "OK";
        if (diag.invalid) {
            health = "INVALID";
        } else if (diag.no_match) {
            health = "NO_URDF_MATCH";
        } else if (diag.out_of_range) {
            health = "OUT_OF_RANGE";
        }

        record_file_ << now_sec << "," << sample.group << "," << sample.name << ",";
        write_value(sample.has_position, sample.position);
        record_file_ << ",";
        write_value(sample.has_velocity, sample.velocity);
        record_file_ << ",";
        write_value(sample.has_effort, sample.effort);
        record_file_ << ",";
        write_value(sample.has_current, sample.current);
        record_file_ << "," << (diag.invalid ? 1 : 0) << "," << (diag.no_match ? 1 : 0) << ","
                     << (diag.out_of_range ? 1 : 0) << "," << health << "\n";

        recorded_rows_++;
    }

    if (recorded_rows_ % 200 == 0) {
        record_file_.flush();
    }
}

int RobotViewerApp::run() {
    if (!initWindow()) {
        return -1;
    }
    if (!initImGui()) {
        return -1;
    }
    if (!initShader()) {
        return -1;
    }
    if (!initScene()) {
        return -1;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGuiIO& io = ImGui::GetIO();
        processCameraInput(io.WantCaptureMouse);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int w = 0;
        int h = 0;
        glfwGetFramebufferSize(window_, &w, &h);

        int min_render_width  = 200;
        int max_sidebar_width = std::max(260, w - min_render_width);
        int min_sidebar_width = std::min(420, max_sidebar_width);
        side_panel_width_     = std::clamp(side_panel_width_, min_sidebar_width, max_sidebar_width);

        int active_sidebar_width = sidebar_collapsed_ ? collapsed_sidebar_width_ : side_panel_width_;
        int render_width         = std::max(200, w - active_sidebar_width);
        int render_height        = h;

        double now_sec       = nowSec();
        uint64_t msg_count   = sensor_subscriber_.messageCount();
        double data_age      = sensor_subscriber_.messageAgeSec();
        bool data_fresh      = sensor_subscriber_.hasRecentData(config_.ui.stale_timeout_seconds);
        latest_sensor_samples_ = sensor_subscriber_.latestSamples();
        updateInputRate(msg_count, now_sec);

        std::unordered_map<std::string, JointDiagState> diag_map;
        diag_map.reserve(latest_sensor_samples_.size());
        int invalid_count   = 0;
        int out_range_count = 0;
        int no_match_count  = 0;
        int shown_count     = 0;

        for (const auto& sample : latest_sensor_samples_) {
            if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
                continue;
            }

            shown_count++;
            JointDiagState diag = evaluateJoint(sample);
            const std::string key = composeJointKey(sample);
            diag_map[key] = diag;

            if (diag.invalid) {
                invalid_count++;
            } else if (diag.no_match) {
                no_match_count++;
            } else if (diag.out_of_range) {
                out_range_count++;
            }
            updateAlarmForJoint(sample, diag, now_sec);

            auto append_metric = [&](bool has_value, double value, const char* suffix) {
                if (!has_value || !std::isfinite(value)) {
                    return;
                }
                std::string key = sample.group + "/" + sample.name + "/" + suffix;
                auto& series     = waveform_history_[key];
                series.push_back(static_cast<float>(value));
                while (static_cast<int>(series.size()) > waveform_history_size_) {
                    series.pop_front();
                }
            };

            append_metric(sample.has_position, sample.position, "position");
            append_metric(sample.has_velocity, sample.velocity, "velocity");
            append_metric(sample.has_effort, sample.effort, "effort");
            append_metric(sample.has_current, sample.current, "current");
        }

        if (recording_) {
            recordCurrentSamples(now_sec, diag_map);
        }

        if (use_sensor_to_drive_robot_ && !latest_sensor_samples_.empty()) {
            scene_.applyJointSamples(latest_sensor_samples_, only_show_master_arm_groups_);
        }

        glViewport(0, 0, render_width, render_height);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shader_);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)render_width / (float)render_height, 0.01f, 100.0f);
        glm::mat4 view       = camera_.viewMatrix();
        glUniformMatrix4fv(glGetUniformLocation(shader_, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shader_, "view"), 1, GL_FALSE, glm::value_ptr(view));

        scene_.setFixedBaseMode(fix_base_like_mujoco_);
        scene_.updateTransforms();
        scene_.draw(shader_);

        glViewport(w - active_sidebar_width, 0, active_sidebar_width, h);
        glDisable(GL_DEPTH_TEST);
        ImGui::SetNextWindowPos(ImVec2(w - active_sidebar_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(active_sidebar_width, h));

        if (sidebar_collapsed_) {
            ImGui::Begin("侧边栏折叠按钮", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);
            ImGui::SetCursorPos(ImVec2(7, 10));
            if (ImGui::ArrowButton("##expand_sidebar", ImGuiDir_Left)) {
                sidebar_collapsed_ = false;
            }
            ImGui::End();
        } else {
            ImGui::Begin("机器人控制", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

            if (ImGui::ArrowButton("##collapse_sidebar", ImGuiDir_Right)) {
                sidebar_collapsed_ = true;
            }
            ImGui::SameLine();
            ImGui::Text("遥操作主臂传感器监控");

            ImGui::PushItemWidth(-1.0f);
            ImGui::DragInt("侧边栏宽度", &side_panel_width_, config_.ui.sidebar_width_drag_speed, min_sidebar_width,
                           max_sidebar_width, "%d px");
            ImGui::PopItemWidth();
            ImGui::Separator();

            ImGui::Text("Topic：");
            ImGui::TextWrapped("%s", config_.sensor.topic.c_str());

            if (!sensor_ready_) {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "传感器订阅初始化失败。");
                use_sensor_to_drive_robot_ = false;
            } else if (msg_count == 0) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "状态：等待数据");
            } else if (!data_fresh) {
                ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "状态：数据陈旧（%.3f s 前）", data_age);
            } else {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "状态：接收中");
            }
            ImGui::Text("消息计数：%llu", static_cast<unsigned long long>(msg_count));

            ImGui::Checkbox("使用传感器数据驱动机器人姿态", &use_sensor_to_drive_robot_);
            if (!sensor_ready_) {
                use_sensor_to_drive_robot_ = false;
            }
            ImGui::Checkbox("仅显示左右臂关节组", &only_show_master_arm_groups_);
            ImGui::Checkbox("固定底座（Mujoco 风格）", &fix_base_like_mujoco_);

            ImGui::Separator();
            ImGui::Text("健康总览");
            ImGui::Text("输入频率：%.1f Hz", input_rate_hz_);
            ImGui::Text("数据时延：%.3f s", data_age >= 0.0 ? data_age : -1.0);

            int health_penalty = invalid_count * 25 + out_range_count * 15 + no_match_count * 10 + (!data_fresh ? 20 : 0);
            int health_score   = std::clamp(100 - health_penalty, 0, 100);
            const char* health_state = "健康";
            ImVec4 health_state_color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
            if (health_score < 60) {
                health_state       = "严重";
                health_state_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
            } else if (health_score < 85) {
                health_state       = "告警";
                health_state_color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
            }
            ImGui::Text("健康评分：%d / 100", health_score);
            ImGui::TextColored(health_state_color, "状态：%s", health_state);

            ImGui::Separator();
            ImGui::Text("会话工具");
            if (ImGui::Button("抓取基线")) {
                baseline_position_rad_.clear();
                for (const auto& sample : latest_sensor_samples_) {
                    if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
                        continue;
                    }
                    if (sample.has_position && std::isfinite(sample.position)) {
                        baseline_position_rad_[composeJointKey(sample)] = sample.position;
                    }
                }
                baseline_ready_ = !baseline_position_rad_.empty();
                baseline_capture_s = now_sec;
            }
            ImGui::SameLine();
            if (ImGui::Button("清除基线")) {
                baseline_position_rad_.clear();
                baseline_ready_ = false;
            }

            if (!recording_) {
                if (ImGui::Button("开始录制 CSV")) {
                    startRecording(now_sec);
                }
            } else {
                if (ImGui::Button("停止录制 CSV")) {
                    stopRecording();
                }
            }

            if (baseline_ready_) {
                ImGui::Text("基线关节数：%d（%.1f s 前抓取）", static_cast<int>(baseline_position_rad_.size()),
                            now_sec - baseline_capture_s);
            } else {
                ImGui::TextDisabled("尚未抓取基线。");
            }

            if (!record_error_.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "录制错误：%s", record_error_.c_str());
            } else if (recording_) {
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "录制状态：进行中（%llu 行）",
                                   static_cast<unsigned long long>(recorded_rows_));
                ImGui::TextWrapped("%s", record_file_path_.c_str());
            } else {
                ImGui::Text("录制状态：已停止");
                if (!record_file_path_.empty()) {
                    ImGui::TextWrapped("最近文件：%s", record_file_path_.c_str());
                }
            }

            ImGui::Separator();
            ImGui::Text("关节诊断");

            ImGui::Text("显示：%d  无效：%d  超限：%d  无模型匹配：%d", shown_count, invalid_count,
                        out_range_count, no_match_count);

            ImGuiTableFlags table_flags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
            if (ImGui::BeginTable("sensor_joint_table", 10, table_flags, ImVec2(-FLT_MIN, 280.0f))) {
                ImGui::TableSetupColumn("关节组");
                ImGui::TableSetupColumn("关节");
                ImGui::TableSetupColumn("位置(rad)");
                ImGui::TableSetupColumn("位置(deg)");
                ImGui::TableSetupColumn("偏差(deg)");
                ImGui::TableSetupColumn("速度");
                ImGui::TableSetupColumn("力矩");
                ImGui::TableSetupColumn("电流");
                ImGui::TableSetupColumn("限位(rad)");
                ImGui::TableSetupColumn("健康");
                ImGui::TableHeadersRow();

                for (const auto& sample : latest_sensor_samples_) {
                    if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
                        continue;
                    }

                    JointDiagState diag;
                    auto diag_it = diag_map.find(composeJointKey(sample));
                    if (diag_it != diag_map.end()) {
                        diag = diag_it->second;
                    } else {
                        diag = evaluateJoint(sample);
                    }

                    const char* health_text = "正常";
                    ImVec4 health_color     = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
                    if (diag.invalid) {
                        health_text  = "无效";
                        health_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
                    } else if (diag.no_match) {
                        health_text  = "无URDF匹配";
                        health_color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
                    } else if (diag.out_of_range) {
                        health_text  = "超限";
                        health_color = ImVec4(1.0f, 0.5f, 0.2f, 1.0f);
                    }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(sample.group.c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(sample.name.c_str());

                    ImGui::TableSetColumnIndex(2);
                    if (sample.has_position && std::isfinite(sample.position)) {
                        ImGui::Text("%.4f", sample.position);
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(3);
                    if (sample.has_position && std::isfinite(sample.position)) {
                        ImGui::Text("%.2f", glm::degrees(static_cast<float>(sample.position)));
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(4);
                    bool has_delta = false;
                    float delta_deg = 0.0f;
                    auto baseline_it = baseline_position_rad_.find(composeJointKey(sample));
                    if (baseline_ready_ && baseline_it != baseline_position_rad_.end() &&
                        sample.has_position && std::isfinite(sample.position)) {
                        delta_deg = glm::degrees(static_cast<float>(sample.position - baseline_it->second));
                        has_delta = true;
                    }
                    if (has_delta) {
                        if (std::abs(delta_deg) > baseline_warn_deg_) {
                            ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 1.0f), "%.2f", delta_deg);
                        } else {
                            ImGui::Text("%.2f", delta_deg);
                        }
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(5);
                    if (sample.has_velocity && std::isfinite(sample.velocity)) {
                        ImGui::Text("%.4f", sample.velocity);
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(6);
                    if (sample.has_effort && std::isfinite(sample.effort)) {
                        ImGui::Text("%.4f", sample.effort);
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(7);
                    if (sample.has_current && std::isfinite(sample.current)) {
                        ImGui::Text("%.4f", sample.current);
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(8);
                    if (diag.has_info) {
                        ImGui::Text("[%.2f, %.2f]", diag.min_angle, diag.max_angle);
                    } else {
                        ImGui::TextUnformatted("-");
                    }

                    ImGui::TableSetColumnIndex(9);
                    ImGui::TextColored(health_color, "%s", health_text);
                }

                ImGui::EndTable();
            }

            ImGui::Separator();
            ImGui::Text("关节波形");

            std::map<std::string, std::vector<std::string>> group_joint_map;
            for (const auto& sample : latest_sensor_samples_) {
                if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
                    continue;
                }

                auto& joints = group_joint_map[sample.group];
                if (std::find(joints.begin(), joints.end(), sample.name) == joints.end()) {
                    joints.push_back(sample.name);
                }
            }

            if (group_joint_map.empty()) {
                ImGui::TextDisabled("当前没有可用于绘图的关节数据。");
            } else {
                if (group_joint_map.find(selected_wave_group_) == group_joint_map.end()) {
                    selected_wave_group_ = group_joint_map.begin()->first;
                }

                const auto& selectable_joints = group_joint_map[selected_wave_group_];
                if (std::find(selectable_joints.begin(), selectable_joints.end(), selected_wave_joint_) ==
                    selectable_joints.end()) {
                    selected_wave_joint_ = selectable_joints.empty() ? std::string() : selectable_joints.front();
                }

                if (selected_wave_metric_ < 0 || selected_wave_metric_ > 3) {
                    selected_wave_metric_ = 0;
                }

                ImGui::PushItemWidth(-1.0f);
                if (ImGui::BeginCombo("波形关节组", selected_wave_group_.c_str())) {
                    for (const auto& [group_name, joints] : group_joint_map) {
                        (void)joints;
                        bool selected = (group_name == selected_wave_group_);
                        if (ImGui::Selectable(group_name.c_str(), selected)) {
                            selected_wave_group_ = group_name;
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }

                const auto& joints_for_selected_group = group_joint_map[selected_wave_group_];
                if (std::find(joints_for_selected_group.begin(), joints_for_selected_group.end(), selected_wave_joint_) ==
                    joints_for_selected_group.end()) {
                    selected_wave_joint_ = joints_for_selected_group.empty() ? std::string() : joints_for_selected_group.front();
                }
                const char* selected_joint_preview =
                    selected_wave_joint_.empty() ? "(none)" : selected_wave_joint_.c_str();
                if (ImGui::BeginCombo("波形关节", selected_joint_preview)) {
                    for (const auto& joint_name : joints_for_selected_group) {
                        bool selected = (joint_name == selected_wave_joint_);
                        if (ImGui::Selectable(joint_name.c_str(), selected)) {
                            selected_wave_joint_ = joint_name;
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }

                ImGui::Combo("波形指标", &selected_wave_metric_, kWaveMetricLabels, IM_ARRAYSIZE(kWaveMetricLabels));
                ImGui::PopItemWidth();

                if (selected_wave_joint_.empty()) {
                    ImGui::TextDisabled("该分组下无可选关节。");
                } else {
                    std::string waveform_key =
                        selected_wave_group_ + "/" + selected_wave_joint_ + "/" + WaveMetricKeySuffix(selected_wave_metric_);
                    auto it = waveform_history_.find(waveform_key);
                    if (it == waveform_history_.end() || it->second.empty()) {
                        ImGui::TextDisabled("正在等待该指标的波形数据...");
                    } else {
                        std::vector<float> plot_values(it->second.begin(), it->second.end());
                        float min_value = *std::min_element(plot_values.begin(), plot_values.end());
                        float max_value = *std::max_element(plot_values.begin(), plot_values.end());
                        if (std::abs(max_value - min_value) < 1e-6f) {
                            min_value -= 1e-3f;
                            max_value += 1e-3f;
                        }

                        DrawWaveformWithAxes("##joint_waveform_plot_axes", plot_values, min_value, max_value,
                                             ImVec2(-FLT_MIN, waveform_plot_height_));
                        ImGui::Text("当前值：%.4f  最小：%.4f  最大：%.4f  样本数：%d", plot_values.back(), min_value,
                                    max_value, static_cast<int>(plot_values.size()));
                    }
                }
            }

            ImGui::Separator();
            ImGui::Text("告警中心");
            int active_alarm_count = 0;
            for (const auto& [key, alarm] : alarms_) {
                (void)key;
                if (alarm.active) {
                    active_alarm_count++;
                }
            }
            ImGui::Text("活跃告警：%d / 总告警：%d", active_alarm_count, static_cast<int>(alarms_.size()));

            ImGui::Checkbox("仅显示活跃告警", &show_only_active_alarms_);
            if (ImGui::Button("确认全部活跃告警")) {
                for (auto& [key, alarm] : alarms_) {
                    (void)key;
                    if (alarm.active) {
                        alarm.acknowledged = true;
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("清理已恢复告警")) {
                for (auto it = alarms_.begin(); it != alarms_.end();) {
                    if (!it->second.active) {
                        bad_frame_streak_.erase(it->first);
                        it = alarms_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }

            std::vector<std::pair<std::string, AlarmEntry*>> alarm_rows;
            alarm_rows.reserve(alarms_.size());
            for (auto& [key, alarm] : alarms_) {
                alarm_rows.push_back({key, &alarm});
            }
            std::sort(alarm_rows.begin(), alarm_rows.end(),
                      [](const auto& a, const auto& b) {
                          if (a.second->active != b.second->active) {
                              return a.second->active > b.second->active;
                          }
                          return a.second->last_seen_s > b.second->last_seen_s;
                      });

            ImGuiTableFlags alarm_table_flags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
            if (ImGui::BeginTable("alarm_table", 7, alarm_table_flags, ImVec2(-FLT_MIN, 180.0f))) {
                ImGui::TableSetupColumn("状态");
                ImGui::TableSetupColumn("关节");
                ImGui::TableSetupColumn("原因");
                ImGui::TableSetupColumn("次数");
                ImGui::TableSetupColumn("首次(s)");
                ImGui::TableSetupColumn("最近(s)");
                ImGui::TableSetupColumn("确认");
                ImGui::TableHeadersRow();

                for (const auto& [alarm_key, alarm_ptr] : alarm_rows) {
                    const AlarmEntry& alarm = *alarm_ptr;
                    if (show_only_active_alarms_ && !alarm.active) {
                        continue;
                    }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    if (alarm.active) {
                        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "活跃");
                    } else {
                        ImGui::TextColored(ImVec4(0.65f, 0.65f, 0.65f, 1.0f), "已恢复");
                    }

                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s/%s", alarm.group.c_str(), alarm.joint.c_str());

                    ImGui::TableSetColumnIndex(2);
                    const char* reason_text = alarm.reason.c_str();
                    if (alarm.reason == "OUT_OF_RANGE") {
                        reason_text = "超限";
                    } else if (alarm.reason == "INVALID") {
                        reason_text = "无效数据";
                    } else if (alarm.reason == "NO_URDF_MATCH") {
                        reason_text = "无URDF匹配";
                    }
                    ImGui::TextUnformatted(reason_text);

                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%d", alarm.trigger_count);

                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%.1f", alarm.first_seen_s);

                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%.1f", alarm.last_seen_s);

                    ImGui::TableSetColumnIndex(6);
                    if (!alarm.active) {
                        ImGui::TextUnformatted("-");
                    } else if (alarm.acknowledged) {
                        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "已确认");
                    } else {
                        std::string btn_id = "确认##" + alarm_key;
                        if (ImGui::SmallButton(btn_id.c_str())) {
                            auto it = alarms_.find(alarm_key);
                            if (it != alarms_.end()) {
                                it->second.acknowledged = true;
                            }
                        }
                    }
                }
                ImGui::EndTable();
            }

            ImGui::Separator();
            ImGui::Text("手动模式（本地调试）");
            if (use_sensor_to_drive_robot_) {
                ImGui::TextDisabled("请先关闭“传感器驱动”后再使用本地滑条。");
            } else {
                auto joints = scene_.getJointInfos();
                for (const auto& joint : joints) {
                    if (!joint.revolute) {
                        continue;
                    }
                    if (fix_base_like_mujoco_ && IsBaseMotionJointName(joint.name)) {
                        continue;
                    }

                    float value = joint.position;
                    if (ImGui::SliderAngle(joint.name.c_str(), &value, glm::degrees(joint.min_angle), glm::degrees(joint.max_angle))) {
                        scene_.setJointPositionByName(joint.name, value);
                    }
                }
            }

            ImGui::Separator();
            ImGui::Text("相机操作：");
            ImGui::Text("左键拖动：旋转（RViz Orbit）");
            ImGui::Text("中键拖动 或 Shift+左键：平移");
            ImGui::Text("右键拖动：Dolly 缩放");
            ImGui::Text("滚轮：缩放");

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glViewport(0, 0, w, h);
        glEnable(GL_DEPTH_TEST);

        glfwSwapBuffers(window_);
    }

    return 0;
}

}  // namespace omnilink::teleop_viewer
