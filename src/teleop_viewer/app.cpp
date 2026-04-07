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

const char* JoyButtonStateText(int status) {
    switch (status) {
        case 0:
            return "UP";
        case 1:
            return "DOWN";
        case 2:
            return "LONG";
        default:
            return "UNKNOWN";
    }
}

ImVec4 JoyButtonStateColor(int status) {
    if (status == 1 || status == 2) {
        return ImVec4(0.25f, 0.95f, 0.35f, 1.0f);
    }
    return ImVec4(0.78f, 0.78f, 0.78f, 1.0f);
}

const char* OmnilinkStateNameZh(const std::string& name) {
    if (name == "TeleopDeviceConnection") return "遥操设备连接";
    if (name == "LeftArmOperationState") return "左臂状态";
    if (name == "RightArmOperationState") return "右臂状态";
    if (name == "HeadOperationState") return "头部状态";
    if (name == "LegOperationState") return "腿部状态";
    if (name == "ChassisOperationState") return "底盘状态";
    if (name == "LeftGripperOperationState") return "左夹爪状态";
    if (name == "RightGripperOperationState") return "右夹爪状态";
    if (name == "RobotStates") return "机器人总状态";
    return "未定义状态";
}

const char* OmnilinkStateValueZh(const std::string& state_name, int status) {
    if (state_name == "TeleopDeviceConnection") {
        if (status == 1) return "在线";
        if (status == 0) return "离线";
        if (status == 3) return "急停";
        return "未知";
    }
    if (state_name == "RobotStates") {
        if (status == 0) return "未连接";
        if (status == 1) return "连接中";
        if (status == 2) return "连接丢失";
        if (status == 3) return "错误";
        return "未知";
    }
    if (state_name == "LeftArmOperationState" || state_name == "RightArmOperationState") {
        if (status == 0) return "锁定";
        if (status == 1) return "关节同步";
        if (status == 2) return "关节接管";
        if (status == 3) return "笛卡尔接管";
        return "未知";
    }
    if (state_name == "HeadOperationState") {
        return status == 0 ? "无操作" : (status == 1 ? "遥操作中" : "未知");
    }
    if (state_name == "LegOperationState") {
        if (status == 0) return "无操作";
        if (status == 1) return "高度调整";
        if (status == 2) return "前后调整";
        if (status == 3) return "腰部调整";
        return "未知";
    }
    if (state_name == "ChassisOperationState") {
        if (status == 0) return "无操作";
        if (status == 1) return "线速度控制";
        if (status == 2) return "旋转控制";
        if (status == 3) return "线旋混合";
        return "未知";
    }
    if (state_name == "LeftGripperOperationState" || state_name == "RightGripperOperationState") {
        return status == 0 ? "无操作" : (status == 1 ? "宽度同步" : "未知");
    }
    return "未知";
}

ImVec4 OmnilinkStateColor(const std::string& state_name, int status) {
    if (state_name == "TeleopDeviceConnection" || state_name == "RobotStates") {
        if (status == 1) return ImVec4(0.35f, 1.0f, 0.45f, 1.0f);
        if (status == 0) return ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
        return ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
    }
    if (status == 0) {
        return ImVec4(0.72f, 0.72f, 0.75f, 1.0f);
    }
    return ImVec4(0.35f, 1.0f, 0.45f, 1.0f);
}

std::string FixCmdToStateName(const std::string& cmd) {
    if (cmd == "fix_height") return "LegOperationState";
    if (cmd == "head_angle_fix") return "HeadOperationState";
    if (cmd == "chassis_fix") return "ChassisOperationState";
    if (cmd == "left_arm_fix") return "LeftArmOperationState";
    if (cmd == "right_arm_fix") return "RightArmOperationState";
    if (cmd == "left_gripper_fix") return "LeftGripperOperationState";
    if (cmd == "right_gripper_fix") return "RightGripperOperationState";
    return std::string();
}

void ApplyTeleopVisualStyle() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.ChildRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.WindowPadding = ImVec2(12.0f, 10.0f);
    style.FramePadding = ImVec2(8.0f, 6.0f);
    style.ItemSpacing = ImVec2(8.0f, 7.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 5.0f);
    style.IndentSpacing = 16.0f;
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.23f, 0.30f, 0.37f, 0.85f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.14f, 0.18f, 0.24f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.19f, 0.25f, 0.32f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.22f, 0.29f, 0.38f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.09f, 0.13f, 0.18f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.12f, 0.18f, 0.25f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.17f, 0.25f, 0.33f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.33f, 0.44f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.24f, 0.36f, 0.47f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.16f, 0.30f, 0.43f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.21f, 0.39f, 0.54f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.14f, 0.27f, 0.37f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.22f, 0.89f, 0.80f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.23f, 0.76f, 0.92f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.18f, 0.63f, 0.80f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.24f, 0.31f, 0.38f, 0.85f);
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.18f, 0.48f, 0.62f, 0.40f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.25f, 0.63f, 0.79f, 0.78f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.21f, 0.56f, 0.72f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.13f, 0.19f, 0.25f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.29f, 0.40f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.17f, 0.27f, 0.36f, 1.00f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.11f, 0.14f, 0.19f, 1.00f);
}

void DrawStatusBadge(const char* id, const char* label, const ImVec4& color) {
    ImGui::PushID(id);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 t = ImGui::CalcTextSize(label);
    ImVec2 size(t.x + 12.0f, t.y + 6.0f);
    ImGui::InvisibleButton("##badge", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
    ImU32 fill = ImColor(color.x, color.y, color.z, 0.28f);
    ImU32 border = ImColor(color.x, color.y, color.z, 0.80f);
    dl->AddRectFilled(p0, p1, fill, 8.0f);
    dl->AddRect(p0, p1, border, 8.0f, 0, 1.2f);
    dl->AddText(ImVec2(p0.x + 6.0f, p0.y + 3.0f), ImColor(color), label);
    ImGui::PopID();
}

void DrawMiniStatCard(const char* id, const char* title, const std::string& value, const ImVec4& accent, float width) {
    ImGui::PushID(id);
    ImGui::BeginChild("##mini_stat", ImVec2(width, 58.0f), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
    ImVec2 p0 = ImGui::GetItemRectMin();
    ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(ImVec2(p0.x + 1.0f, p0.y + 1.0f), ImVec2(p0.x + 4.0f, p1.y - 1.0f), ImColor(accent), 3.0f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 3.0f);
    ImGui::TextDisabled("%s", title);
    ImGui::TextColored(ImVec4(0.90f, 0.96f, 1.0f, 1.0f), "%s", value.c_str());
    ImGui::EndChild();
    ImGui::PopID();
}

void PushSectionHeaderStyle() {
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.15f, 0.24f, 0.33f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.21f, 0.32f, 0.44f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.19f, 0.30f, 0.40f, 1.0f));
}

void PopSectionHeaderStyle() { ImGui::PopStyleColor(3); }

double GetAxisValue(const std::map<std::string, double>& axis_map, const std::vector<std::string>& names, bool* found = nullptr) {
    for (const auto& n : names) {
        auto it = axis_map.find(n);
        if (it != axis_map.end() && std::isfinite(it->second)) {
            if (found) {
                *found = true;
            }
            return it->second;
        }
    }
    if (found) {
        *found = false;
    }
    return 0.0;
}

void DrawJoyStickPad(const char* id, float x, float y, ImVec2 size = ImVec2(130, 130)) {
    ImGui::InvisibleButton(id, size);
    ImVec2 p_min = ImGui::GetItemRectMin();
    ImVec2 p_max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    ImVec2 center((p_min.x + p_max.x) * 0.5f, (p_min.y + p_max.y) * 0.5f);
    float radius = std::min(size.x, size.y) * 0.42f;

    dl->AddRectFilled(p_min, p_max, IM_COL32(22, 24, 28, 255), 4.0f);
    dl->AddRect(p_min, p_max, IM_COL32(85, 85, 95, 255), 4.0f);
    dl->AddCircle(center, radius, IM_COL32(160, 160, 170, 255), 48, 1.0f);
    dl->AddLine(ImVec2(center.x - radius, center.y), ImVec2(center.x + radius, center.y), IM_COL32(65, 65, 75, 255), 1.0f);
    dl->AddLine(ImVec2(center.x, center.y - radius), ImVec2(center.x, center.y + radius), IM_COL32(65, 65, 75, 255), 1.0f);

    float nx = std::clamp(x, -1.0f, 1.0f);
    float ny = std::clamp(y, -1.0f, 1.0f);
    ImVec2 knob(center.x + nx * radius, center.y - ny * radius);
    dl->AddCircleFilled(knob, 6.5f, IM_COL32(95, 205, 255, 255), 24);
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
    auto_lock_on_critical_fault_ = config_.omnilink_bridge.auto_lock_on_critical_fault;

    for (const auto& name : config_.omnilink_bridge.rc_button_names) {
        rc_virtual_joy_command_[name] = 0;
    }
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
    ApplyTeleopVisualStyle();

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
    joy_ready_ = sensor_ready_ && sensor_subscriber_.startJoy(config_.joy.topic);
    if (config_.omnilink_bridge.enable) {
        omnilink_state_ready_ =
            sensor_ready_ && sensor_subscriber_.startOmnilinkStates(config_.omnilink_bridge.state_topic);
        wbc_ready_ = !config_.omnilink_bridge.enable_wbc_monitor ||
                     (sensor_ready_ && sensor_subscriber_.startWbcInfo(config_.omnilink_bridge.wbc_info_topic));
        robot_error_ready_ = !config_.omnilink_bridge.enable_error_monitor ||
                             (sensor_ready_ && sensor_subscriber_.startRobotErrors(config_.omnilink_bridge.error_topic));
        rc_virtual_joy_ready_ =
            sensor_ready_ && sensor_subscriber_.startRcVirtualJoyPublisher(config_.omnilink_bridge.rc_virtual_joy_topic);
    } else {
        omnilink_state_ready_ = false;
        wbc_ready_ = false;
        robot_error_ready_ = false;
        rc_virtual_joy_ready_ = false;
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

RobotViewerApp::FrameLayout RobotViewerApp::computeFrameLayout() {
    FrameLayout layout;
    glfwGetFramebufferSize(window_, &layout.window_width, &layout.window_height);

    int min_render_width   = 200;
    layout.max_sidebar_width = std::max(260, layout.window_width - min_render_width);
    layout.min_sidebar_width = std::min(420, layout.max_sidebar_width);
    side_panel_width_ = std::clamp(side_panel_width_, layout.min_sidebar_width, layout.max_sidebar_width);

    layout.active_sidebar_width = sidebar_collapsed_ ? collapsed_sidebar_width_ : side_panel_width_;
    layout.render_width  = std::max(200, layout.window_width - layout.active_sidebar_width);
    layout.render_height = layout.window_height;
    return layout;
}

RobotViewerApp::FrameData RobotViewerApp::collectFrameData(double now_sec) {
    FrameData frame;
    frame.now_sec = now_sec;
    frame.msg_count = sensor_subscriber_.messageCount();
    frame.data_age = sensor_subscriber_.messageAgeSec();
    frame.data_fresh = sensor_subscriber_.hasRecentData(config_.ui.stale_timeout_seconds);
    frame.joy_msg_count = sensor_subscriber_.joyMessageCount();
    frame.joy_data_age = sensor_subscriber_.joyMessageAgeSec();
    frame.joy_data_fresh = sensor_subscriber_.joyHasRecentData(config_.ui.stale_timeout_seconds);
    frame.state_msg_count = sensor_subscriber_.stateMessageCount();
    frame.state_data_age = sensor_subscriber_.stateMessageAgeSec();
    frame.state_data_fresh = sensor_subscriber_.stateHasRecentData(config_.ui.stale_timeout_seconds);
    frame.wbc_msg_count = sensor_subscriber_.wbcMessageCount();
    frame.wbc_data_age = sensor_subscriber_.wbcMessageAgeSec();
    frame.wbc_data_fresh = sensor_subscriber_.wbcHasRecentData(config_.ui.stale_timeout_seconds);
    frame.error_msg_count = sensor_subscriber_.errorMessageCount();
    frame.error_data_age = sensor_subscriber_.errorMessageAgeSec();
    frame.error_data_fresh = sensor_subscriber_.errorHasRecentData(config_.ui.stale_timeout_seconds);

    latest_sensor_samples_ = sensor_subscriber_.latestSamples();
    latest_joy_buttons_ = sensor_subscriber_.latestJoyButtons();
    latest_joy_axes_ = sensor_subscriber_.latestJoyAxes();
    latest_state_buttons_ = sensor_subscriber_.latestStateButtons();
    latest_state_axes_ = sensor_subscriber_.latestStateAxes();
    latest_wbc_group_errors_ = sensor_subscriber_.latestWbcGroupErrors();
    latest_wbc_joint_name_count_ = sensor_subscriber_.latestWbcJointNameCount();
    latest_wbc_state_pos_count_ = sensor_subscriber_.latestWbcStatePosCount();
    latest_robot_errors_ = sensor_subscriber_.latestRobotErrors();
    frame.error_active_count = static_cast<int>(latest_robot_errors_.size());

    for (const auto& button : latest_joy_buttons_) {
        auto it = prev_joy_button_status_.find(button.name);
        if (it == prev_joy_button_status_.end() || it->second != button.status) {
            JoyButtonLogEntry entry;
            entry.time_s = now_sec;
            entry.hand = IsLeftHandleKey(button.name) ? "左" : (IsRightHandleKey(button.name) ? "右" : "未知");
            entry.name = button.name;
            entry.status = button.status;
            joy_button_logs_.push_front(std::move(entry));
            while (joy_button_logs_.size() > joy_button_log_limit_) {
                joy_button_logs_.pop_back();
            }
            prev_joy_button_status_[button.name] = button.status;
        }
    }

    updateInputRate(frame.msg_count, now_sec);

    frame.diag_map.reserve(latest_sensor_samples_.size());
    for (const auto& sample : latest_sensor_samples_) {
        if (only_show_master_arm_groups_ && !IsMasterArmGroup(sample.group)) {
            continue;
        }

        frame.shown_count++;
        JointDiagState diag = evaluateJoint(sample);
        const std::string key = composeJointKey(sample);
        frame.diag_map[key] = diag;

        if (diag.invalid) {
            frame.invalid_count++;
        } else if (diag.no_match) {
            frame.no_match_count++;
        } else if (diag.out_of_range) {
            frame.out_range_count++;
        }
        updateAlarmForJoint(sample, diag, now_sec);

        auto append_metric = [&](bool has_value, double value, const char* suffix) {
            if (!has_value || !std::isfinite(value)) {
                return;
            }
            std::string waveform_key = sample.group + "/" + sample.name + "/" + suffix;
            auto& series = waveform_history_[waveform_key];
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
        recordCurrentSamples(now_sec, frame.diag_map);
    }
    if (use_sensor_to_drive_robot_ && !latest_sensor_samples_.empty()) {
        scene_.applyJointSamples(latest_sensor_samples_, only_show_master_arm_groups_);
    }
    return frame;
}

void RobotViewerApp::pushRcCommandLog(double now_sec, const std::string& source, bool ok, int active_locks, const std::string& note) {
    RcCommandLogEntry item;
    item.time_s = now_sec;
    item.source = source;
    item.ok = ok;
    item.active_locks = active_locks;
    item.note = note;
    rc_virtual_joy_logs_.push_front(std::move(item));
    while (rc_virtual_joy_logs_.size() > rc_virtual_joy_log_limit_) {
        rc_virtual_joy_logs_.pop_back();
    }
}

bool RobotViewerApp::publishRcVirtualJoyCommand(const std::string& source, const std::string& note) {
    std::vector<std::pair<std::string, int>> buttons;
    buttons.reserve(config_.omnilink_bridge.rc_button_names.size());
    int active_locks = 0;
    for (const auto& name : config_.omnilink_bridge.rc_button_names) {
        int value = 0;
        auto it = rc_virtual_joy_command_.find(name);
        if (it != rc_virtual_joy_command_.end()) {
            value = it->second;
        }
        if (value == 1) {
            active_locks++;
        }
        buttons.push_back({name, value});
    }

    double now_sec = nowSec();
    bool ok = sensor_subscriber_.publishRcVirtualJoyButtons(buttons);
    if (ok) {
        last_rc_virtual_joy_send_s_ = now_sec;
        rc_virtual_joy_send_count_++;
        rc_virtual_joy_last_error_.clear();
    } else {
        rc_virtual_joy_last_error_ = "发送失败：命令写通道未就绪";
    }
    pushRcCommandLog(now_sec, source, ok, active_locks, note);
    return ok;
}

void RobotViewerApp::renderSceneFrame(const FrameLayout& layout) {
    glViewport(0, 0, layout.render_width, layout.render_height);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shader_);
    glm::mat4 projection =
        glm::perspective(glm::radians(45.0f), (float)layout.render_width / (float)layout.render_height, 0.01f, 100.0f);
    glm::mat4 view = camera_.viewMatrix();
    glUniformMatrix4fv(glGetUniformLocation(shader_, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shader_, "view"), 1, GL_FALSE, glm::value_ptr(view));

    scene_.setFixedBaseMode(fix_base_like_mujoco_);
    scene_.updateTransforms();
    scene_.draw(shader_);
}

int RobotViewerApp::run() {
    if (!initWindow() || !initImGui() || !initShader() || !initScene()) {
        return -1;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGuiIO& io = ImGui::GetIO();
        processCameraInput(io.WantCaptureMouse);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        FrameLayout layout = computeFrameLayout();
        FrameData frame = collectFrameData(nowSec());

        renderSceneFrame(layout);
        renderSidebar(layout, frame);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glViewport(0, 0, layout.window_width, layout.window_height);
        glEnable(GL_DEPTH_TEST);
        glfwSwapBuffers(window_);
    }

    return 0;
}

}  // namespace omnilink::teleop_viewer
