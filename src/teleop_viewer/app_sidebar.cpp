#include "teleop_viewer/app.h"
#include "imgui.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace omnilink::teleop_viewer {
namespace {
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

std::string JoyButtonAliasName(const std::string& raw_name) {
    if (raw_name == "key1L") return "left_combo";
    if (raw_name == "key2L") return "left_record";
    if (raw_name == "key3L") return "left_takeover_switch";
    if (raw_name == "key4L") return "left_view_switch";
    if (raw_name == "key5L") return "left_arm_sync";
    if (raw_name == "key6L") return "left_gripper_active";
    if (raw_name == "key7L") return "left_stop";
    if (raw_name == "key8L") return "left_takeover";
    if (raw_name == "key9L") return "left_reversed";
    if (raw_name == "key1R") return "right_combo";
    if (raw_name == "key2R") return "right_record";
    if (raw_name == "key3R") return "right_takeover_switch";
    if (raw_name == "key4R") return "right_view_switch";
    if (raw_name == "key5R") return "right_arm_sync";
    if (raw_name == "key6R") return "right_gripper_active";
    if (raw_name == "key7R") return "right_stop";
    if (raw_name == "key8R") return "right_takeover";
    if (raw_name == "key9R") return "right_reversed";
    return raw_name;
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


void RobotViewerApp::renderSidebar(const FrameLayout& layout, const FrameData& frame) {
    glViewport(layout.window_width - layout.active_sidebar_width, 0, layout.active_sidebar_width, layout.window_height);
    glDisable(GL_DEPTH_TEST);
    ImGui::SetNextWindowPos(ImVec2(layout.window_width - layout.active_sidebar_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(layout.active_sidebar_width, layout.window_height));

    if (sidebar_collapsed_) {
        ImGui::Begin("侧边栏折叠按钮", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);
        ImGui::SetCursorPos(ImVec2(7, 10));
        if (ImGui::ArrowButton("##expand_sidebar", ImGuiDir_Left)) {
            sidebar_collapsed_ = false;
        }
        ImGui::End();
        return;
    }

    ImGui::Begin("机器人控制", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

    // 左边缘拖拽分割条：按住左右拖拽调整侧边栏宽度（类似常见IDE侧栏）
    {
        ImGuiIO& io = ImGui::GetIO();
        const float grip_width = 8.0f;
        ImVec2 window_pos = ImGui::GetWindowPos();
        ImVec2 window_size = ImGui::GetWindowSize();
        ImVec2 grip_min(window_pos.x, window_pos.y);
        ImVec2 grip_max(window_pos.x + grip_width, window_pos.y + window_size.y);

        ImGui::SetCursorScreenPos(grip_min);
        ImGui::InvisibleButton("##sidebar_resize_grip", ImVec2(grip_width, window_size.y));
        bool grip_hovered = ImGui::IsItemHovered();
        bool grip_active = ImGui::IsItemActive();
        if (grip_hovered || grip_active) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
        if (grip_active) {
            side_panel_width_ = std::clamp(side_panel_width_ - static_cast<int>(io.MouseDelta.x), layout.min_sidebar_width,
                                           layout.max_sidebar_width);
        }

        ImU32 grip_color = grip_active ? IM_COL32(120, 200, 255, 220)
                                       : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
        ImGui::GetWindowDrawList()->AddRectFilled(grip_min, grip_max, grip_color, 2.0f);
        ImGui::SetCursorPosY(8.0f);
    }

    if (ImGui::ArrowButton("##collapse_sidebar", ImGuiDir_Right)) {
        sidebar_collapsed_ = true;
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.40f, 0.86f, 1.00f, 1.00f), "遥操作主臂传感器监控");

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.11f, 0.15f, 0.21f, 0.90f));
    ImGui::BeginChild("teleop_header_card", ImVec2(0.0f, 56.0f), true, ImGuiWindowFlags_NoScrollbar);
    ImGui::TextColored(ImVec4(0.84f, 0.94f, 1.0f, 1.0f), "SingoriX Teleop Console");
    ImGui::TextDisabled("Realtime Sensor + OmniLink Command Center");
    ImGui::EndChild();
    ImGui::PopStyleColor();

    std::string sensor_kpi = !sensor_ready_
                                 ? "初始化失败"
                                 : (frame.msg_count == 0 ? "等待数据" : (frame.data_fresh ? "在线" : "陈旧"));
    std::string joy_kpi = !joy_ready_
                              ? "初始化失败"
                              : (frame.joy_msg_count == 0 ? "等待数据" : (frame.joy_data_fresh ? "在线" : "陈旧"));
    std::ostringstream cmd_kpi_ss;
    cmd_kpi_ss << rc_virtual_joy_send_count_ << " 次";
    std::string cmd_kpi = rc_virtual_joy_ready_ ? cmd_kpi_ss.str() : "通道未就绪";

    float avail_w = ImGui::GetContentRegionAvail().x;
    float card_gap = 8.0f;
    float card_w = (avail_w - 2.0f * card_gap) / 3.0f;
    if (card_w < 90.0f) {
        card_w = 90.0f;
    }
    DrawMiniStatCard("kpi_sensor", "传感器", sensor_kpi, ImVec4(0.35f, 0.85f, 1.0f, 1.0f), card_w);
    ImGui::SameLine();
    DrawMiniStatCard("kpi_joy", "手柄消息", joy_kpi, ImVec4(0.32f, 0.95f, 0.65f, 1.0f), card_w);
    ImGui::SameLine();
    DrawMiniStatCard("kpi_cmd", "命令发送", cmd_kpi, ImVec4(0.98f, 0.72f, 0.27f, 1.0f), card_w);

    ImGui::PushItemWidth(-1.0f);
    ImGui::DragInt("侧边栏宽度", &side_panel_width_, config_.ui.sidebar_width_drag_speed, layout.min_sidebar_width,
                   layout.max_sidebar_width, "%d px");
    ImGui::PopItemWidth();
    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool sensor_section_open = ImGui::CollapsingHeader("传感器与渲染###section_sensor", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (sensor_section_open) {
        ImGui::Text("Topic：");
        ImGui::TextWrapped("%s", config_.sensor.topic.c_str());

        if (!sensor_ready_) {
            ImGui::Text("状态：");
            ImGui::SameLine();
            DrawStatusBadge("sensor_init", "初始化失败", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
            use_sensor_to_drive_robot_ = false;
        } else if (frame.msg_count == 0) {
            ImGui::Text("状态：");
            ImGui::SameLine();
            DrawStatusBadge("sensor_wait", "等待数据", ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
        } else if (!frame.data_fresh) {
            ImGui::Text("状态：");
            ImGui::SameLine();
            DrawStatusBadge("sensor_stale", "数据陈旧", ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
            ImGui::SameLine();
            ImGui::TextDisabled("(%.3f s 前)", frame.data_age);
        } else {
            ImGui::Text("状态：");
            ImGui::SameLine();
            DrawStatusBadge("sensor_ok", "接收中", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
        }
        ImGui::Text("消息计数：%llu", static_cast<unsigned long long>(frame.msg_count));

        ImGui::Checkbox("使用传感器数据驱动机器人姿态", &use_sensor_to_drive_robot_);
        if (!sensor_ready_) {
            use_sensor_to_drive_robot_ = false;
        }
        ImGui::Checkbox("仅显示左右臂关节组", &only_show_master_arm_groups_);
        ImGui::Checkbox("固定底座（Mujoco 风格）", &fix_base_like_mujoco_);
    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool omnilink_section_open = ImGui::CollapsingHeader("OmniLink 控制中心###section_omnilink", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (omnilink_section_open) {
        ImGui::Text("命令 Topic：");
        ImGui::TextWrapped("%s", config_.omnilink_bridge.rc_virtual_joy_topic.c_str());
        ImGui::Text("状态 Topic：");
        ImGui::TextWrapped("%s", config_.omnilink_bridge.state_topic.c_str());
        ImGui::Text("WBC Topic：");
        ImGui::TextWrapped("%s", config_.omnilink_bridge.wbc_info_topic.c_str());
        ImGui::Text("错误 Topic：");
        ImGui::TextWrapped("%s", config_.omnilink_bridge.error_topic.c_str());

        std::map<std::string, int> omnilink_state_map;
        for (const auto& button : latest_state_buttons_) {
            omnilink_state_map[button.name] = button.status;
        }
        bool wbc_monitor_enabled = config_.omnilink_bridge.enable_wbc_monitor;
        bool error_monitor_enabled = config_.omnilink_bridge.enable_error_monitor;

        auto stop_inspect = [&](const std::string& reason) {
            rc_inspect_running_ = false;
            rc_inspect_lock_phase_ = true;
            rc_inspect_index_ = 0;
            rc_inspect_step_begin_s_ = -1.0;
            rc_inspect_last_message_ = reason;
            rc_inspect_last_finish_s_ = frame.now_sec;
        };
        auto start_inspect = [&]() {
            rc_inspect_order_.clear();
            rc_inspect_results_.clear();
            for (const auto& cmd : config_.omnilink_bridge.rc_button_names) {
                if (!FixCmdToStateName(cmd).empty()) {
                    rc_inspect_order_.push_back(cmd);
                    rc_inspect_results_[cmd] = RcInspectResult{};
                }
            }
            if (rc_inspect_order_.empty()) {
                stop_inspect("自动巡检未启动：没有可校验的命令");
                return;
            }
            rc_inspect_running_ = true;
            rc_inspect_lock_phase_ = true;
            rc_inspect_index_ = 0;
            rc_inspect_step_begin_s_ = -1.0;
            rc_inspect_last_message_.clear();
        };

        if (!config_.omnilink_bridge.enable) {
            stop_inspect("自动巡检已停止：桥接功能未启用");
            ImGui::TextDisabled("桥接功能已禁用（omnilink_bridge.enable=false）");
        } else {
            if (rc_inspect_running_ &&
                (!rc_virtual_joy_ready_ || !omnilink_state_ready_ || (wbc_monitor_enabled && !wbc_ready_) ||
                 (error_monitor_enabled && !robot_error_ready_))) {
                stop_inspect("自动巡检已停止：命令或状态通道未就绪");
            }

            if (rc_inspect_running_ && rc_inspect_index_ < rc_inspect_order_.size()) {
                const std::string& current_cmd = rc_inspect_order_[rc_inspect_index_];
                RcInspectResult& current_result = rc_inspect_results_[current_cmd];
                std::string state_name = FixCmdToStateName(current_cmd);
                auto state_it = omnilink_state_map.find(state_name);
                bool state_valid = !state_name.empty() && state_it != omnilink_state_map.end();
                int state_value = state_valid ? state_it->second : -1;

                if (rc_inspect_step_begin_s_ < 0.0) {
                    int target_value = rc_inspect_lock_phase_ ? 1 : 0;
                    rc_virtual_joy_command_[current_cmd] = target_value;
                    rc_virtual_joy_changed_at_s_[current_cmd] = frame.now_sec;
                    if (rc_inspect_lock_phase_) {
                        current_result.attempts++;
                    }

                    std::ostringstream step_note;
                    step_note << (rc_inspect_lock_phase_ ? "lock:" : "unlock:") << current_cmd;
                    if (!publishRcVirtualJoyCommand("auto.inspect", step_note.str())) {
                        stop_inspect("自动巡检已停止：命令发送失败");
                    } else {
                        rc_inspect_step_begin_s_ = frame.now_sec;
                    }
                }

                if (rc_inspect_running_ && rc_inspect_step_begin_s_ >= 0.0) {
                    double phase_elapsed = frame.now_sec - rc_inspect_step_begin_s_;
                    if (rc_inspect_lock_phase_) {
                        bool reached = state_valid && state_value == 0;
                        bool timeout = phase_elapsed >= rc_inspect_timeout_s_;
                        if (reached || timeout) {
                            if (reached) {
                                current_result.pass++;
                                current_result.last_latency_s = phase_elapsed;
                                rc_virtual_joy_changed_at_s_.erase(current_cmd);
                            } else {
                                current_result.timeout++;
                                current_result.last_latency_s = -1.0;
                            }
                            rc_inspect_lock_phase_ = false;
                            rc_inspect_step_begin_s_ = -1.0;
                        }
                    } else if (phase_elapsed >= rc_inspect_step_duration_s_) {
                        rc_inspect_lock_phase_ = true;
                        rc_inspect_step_begin_s_ = -1.0;
                        rc_inspect_index_++;
                        if (rc_inspect_index_ >= rc_inspect_order_.size()) {
                            int total_attempts = 0;
                            int total_pass = 0;
                            int total_timeout = 0;
                            for (const auto& [cmd, result] : rc_inspect_results_) {
                                (void)cmd;
                                total_attempts += result.attempts;
                                total_pass += result.pass;
                                total_timeout += result.timeout;
                            }
                            std::ostringstream done_ss;
                            done_ss << "自动巡检完成：通过 " << total_pass << "/" << total_attempts
                                    << "，超时 " << total_timeout;
                            stop_inspect(done_ss.str());
                        }
                    }
                }
            }

            ImGui::Text("命令通道：");
            ImGui::SameLine();
            if (!rc_virtual_joy_ready_) {
                DrawStatusBadge("cmd_channel", "未就绪", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
            } else {
                DrawStatusBadge("cmd_channel", "就绪", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
            }

            ImGui::Text("状态回传：");
            ImGui::SameLine();
            if (!omnilink_state_ready_) {
                DrawStatusBadge("state_channel", "未就绪", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
            } else if (frame.state_msg_count == 0) {
                DrawStatusBadge("state_channel", "等待数据", ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
            } else if (!frame.state_data_fresh) {
                DrawStatusBadge("state_channel", "数据陈旧", ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
                ImGui::SameLine();
                ImGui::TextDisabled("(%.3f s 前)", frame.state_data_age);
            } else {
                DrawStatusBadge("state_channel", "接收中", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
            }

            if (wbc_monitor_enabled) {
                ImGui::Text("WBC回传：");
                ImGui::SameLine();
                if (!wbc_ready_) {
                    DrawStatusBadge("wbc_channel", "未就绪", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                } else if (frame.wbc_msg_count == 0) {
                    DrawStatusBadge("wbc_channel", "等待数据", ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
                } else if (!frame.wbc_data_fresh) {
                    DrawStatusBadge("wbc_channel", "数据陈旧", ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
                    ImGui::SameLine();
                    ImGui::TextDisabled("(%.3f s 前)", frame.wbc_data_age);
                } else {
                    DrawStatusBadge("wbc_channel", "接收中", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
                }
            } else {
                ImGui::TextDisabled("WBC监控已禁用");
            }

            if (error_monitor_enabled) {
                ImGui::Text("机器人错误：");
                ImGui::SameLine();
                if (!robot_error_ready_) {
                    DrawStatusBadge("err_channel", "未就绪", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                } else if (frame.error_msg_count == 0) {
                    DrawStatusBadge("err_channel", "等待数据", ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
                } else if (frame.error_active_count > 0) {
                    DrawStatusBadge("err_channel", "存在错误", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                } else {
                    DrawStatusBadge("err_channel", "无错误", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
                }
                if (frame.error_data_age >= 0.0) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("(%.3f s 前)", frame.error_data_age);
                }
            } else {
                ImGui::TextDisabled("错误监控已禁用");
            }

            ImGui::Text("状态消息计数：%llu", static_cast<unsigned long long>(frame.state_msg_count));
            if (wbc_monitor_enabled) {
                ImGui::Text("WBC消息计数：%llu", static_cast<unsigned long long>(frame.wbc_msg_count));
            }
            if (error_monitor_enabled) {
                ImGui::Text("错误消息计数：%llu（当前错误项：%d）",
                            static_cast<unsigned long long>(frame.error_msg_count), frame.error_active_count);
            }
            ImGui::Text("命令发送计数：%llu", static_cast<unsigned long long>(rc_virtual_joy_send_count_));
            if (last_rc_virtual_joy_send_s_ >= 0.0) {
                ImGui::Text("最近命令：%.2f s 前", frame.now_sec - last_rc_virtual_joy_send_s_);
            } else {
                ImGui::TextDisabled("最近命令：尚未发送");
            }
            if (!rc_virtual_joy_last_error_.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", rc_virtual_joy_last_error_.c_str());
            }

            ImGui::SeparatorText("控制链路诊断");
            if (wbc_monitor_enabled) {
                std::vector<WbcGroupErrorSample> wbc_rows = latest_wbc_group_errors_;
                std::sort(wbc_rows.begin(), wbc_rows.end(), [](const WbcGroupErrorSample& a, const WbcGroupErrorSample& b) {
                    double a_max = std::max({std::abs(a.pos_norm), std::abs(a.vel_norm), std::abs(a.eff_norm)});
                    double b_max = std::max({std::abs(b.pos_norm), std::abs(b.vel_norm), std::abs(b.eff_norm)});
                    return a_max > b_max;
                });

                if (latest_wbc_joint_name_count_ > 0 || latest_wbc_state_pos_count_ > 0) {
                    bool size_mismatch = latest_wbc_joint_name_count_ != latest_wbc_state_pos_count_;
                    ImGui::Text("WBC关节覆盖：names=%d  state_pos=%d", latest_wbc_joint_name_count_, latest_wbc_state_pos_count_);
                    if (size_mismatch) {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.25f, 1.0f), "（长度不一致）");
                    }
                }

                if (wbc_rows.empty()) {
                    ImGui::TextDisabled("暂无WBC group误差信息。");
                } else {
                    ImGuiTableFlags wbc_flags =
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
                    if (ImGui::BeginTable("wbc_group_error_table", 5, wbc_flags, ImVec2(-FLT_MIN, 140.0f))) {
                        ImGui::TableSetupColumn("分组");
                        ImGui::TableSetupColumn("关节数");
                        ImGui::TableSetupColumn("位置误差范数");
                        ImGui::TableSetupColumn("速度误差范数");
                        ImGui::TableSetupColumn("力矩误差范数");
                        ImGui::TableHeadersRow();
                        for (const auto& row : wbc_rows) {
                            auto color_for = [&](double v) {
                                double av = std::abs(v);
                                if (av >= config_.omnilink_bridge.wbc_norm_danger) {
                                    return ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
                                }
                                if (av >= config_.omnilink_bridge.wbc_norm_warn) {
                                    return ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
                                }
                                return ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
                            };

                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::TextUnformatted(row.group.c_str());
                            ImGui::TableSetColumnIndex(1);
                            ImGui::Text("%d", row.joint_count);
                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextColored(color_for(row.pos_norm), "%.5f", row.pos_norm);
                            ImGui::TableSetColumnIndex(3);
                            ImGui::TextColored(color_for(row.vel_norm), "%.5f", row.vel_norm);
                            ImGui::TableSetColumnIndex(4);
                            ImGui::TextColored(color_for(row.eff_norm), "%.5f", row.eff_norm);
                        }
                        ImGui::EndTable();
                    }
                }
            }

            if (error_monitor_enabled) {
                if (latest_robot_errors_.empty()) {
                    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "当前无机器人错误。");
                } else {
                    ImGuiTableFlags err_flags =
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
                    if (ImGui::BeginTable("robot_error_table", 3, err_flags, ImVec2(-FLT_MIN, 120.0f))) {
                        ImGui::TableSetupColumn("组件");
                        ImGui::TableSetupColumn("错误码");
                        ImGui::TableSetupColumn("描述");
                        ImGui::TableHeadersRow();
                        for (const auto& err : latest_robot_errors_) {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::TextUnformatted(err.component.c_str());
                            ImGui::TableSetColumnIndex(1);
                            ImGui::Text("0x%X", static_cast<unsigned int>(err.code));
                            ImGui::TableSetColumnIndex(2);
                            if (err.description.empty()) {
                                ImGui::TextDisabled("-");
                            } else {
                                ImGui::TextUnformatted(err.description.c_str());
                            }
                        }
                        ImGui::EndTable();
                    }
                }
            }

            ImGui::SeparatorText("安全守护");
            ImGui::Checkbox("关键故障自动全锁定", &auto_lock_on_critical_fault_);
            bool sensor_stale = frame.msg_count > 0 && !frame.data_fresh;
            bool state_stale = frame.state_msg_count > 0 && !frame.state_data_fresh;
            bool wbc_stale = wbc_monitor_enabled && frame.wbc_msg_count > 0 && !frame.wbc_data_fresh;
            bool channel_not_ready = !rc_virtual_joy_ready_ || !omnilink_state_ready_ || (wbc_monitor_enabled && !wbc_ready_) ||
                                     (error_monitor_enabled && !robot_error_ready_);
            bool critical_fault = channel_not_ready || sensor_stale || state_stale || wbc_stale ||
                                  (error_monitor_enabled && frame.error_active_count > 0);
            if (critical_fault) {
                if (auto_lock_on_critical_fault_ && !auto_lock_latched_ && rc_virtual_joy_ready_) {
                    for (const auto& name : config_.omnilink_bridge.rc_button_names) {
                        rc_virtual_joy_command_[name] = 1;
                        rc_virtual_joy_changed_at_s_[name] = frame.now_sec;
                    }
                    std::string reason = "critical_fault";
                    if (sensor_stale) {
                        reason += " sensor_stale";
                    }
                    if (state_stale) {
                        reason += " state_stale";
                    }
                    if (wbc_stale) {
                        reason += " wbc_stale";
                    }
                    if (channel_not_ready) {
                        reason += " channel_not_ready";
                    }
                    if (error_monitor_enabled && frame.error_active_count > 0) {
                        reason += " robot_error";
                    }
                    publishRcVirtualJoyCommand("safety.auto_lock", reason);
                    auto_lock_latched_ = true;
                    auto_lock_reason_ = reason;
                    auto_lock_time_s_ = frame.now_sec;
                }
            } else {
                auto_lock_latched_ = false;
            }

            ImGui::Text("守护状态：%s", critical_fault ? "故障触发中" : "正常");
            if (!auto_lock_reason_.empty()) {
                ImGui::TextWrapped("最近自动动作：%s", auto_lock_reason_.c_str());
                if (auto_lock_time_s_ >= 0.0) {
                    ImGui::TextDisabled("(%.2f s 前)", frame.now_sec - auto_lock_time_s_);
                }
            }
            if (critical_fault && !auto_lock_on_critical_fault_) {
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "检测到关键故障，建议立即执行“全锁定”。");
            }

            ImGui::SeparatorText("自动巡检工具");
            ImGui::SliderFloat("锁定判定超时(s)", &rc_inspect_timeout_s_, 0.2f, 3.0f, "%.1f");
            ImGui::SliderFloat("解锁停留时长(s)", &rc_inspect_step_duration_s_, 0.2f, 3.0f, "%.1f");

            if (!rc_inspect_running_) {
                if (ImGui::Button("开始自动巡检")) {
                    start_inspect();
                }
                ImGui::SameLine();
                if (ImGui::Button("清空巡检统计")) {
                    rc_inspect_results_.clear();
                    rc_inspect_last_message_ = "已清空巡检统计";
                }
            } else if (ImGui::Button("停止自动巡检")) {
                stop_inspect("自动巡检已由用户停止");
            }

            int inspect_attempts = 0;
            int inspect_pass = 0;
            int inspect_timeout = 0;
            for (const auto& [cmd, result] : rc_inspect_results_) {
                (void)cmd;
                inspect_attempts += result.attempts;
                inspect_pass += result.pass;
                inspect_timeout += result.timeout;
            }
            ImGui::Text("巡检统计：通过 %d / %d，超时 %d", inspect_pass, inspect_attempts, inspect_timeout);
            if (!rc_inspect_last_message_.empty()) {
                ImGui::TextWrapped("%s", rc_inspect_last_message_.c_str());
                if (rc_inspect_last_finish_s_ >= 0.0) {
                    ImGui::TextDisabled("(%.2f s 前更新)", frame.now_sec - rc_inspect_last_finish_s_);
                }
            }
            if (!rc_inspect_order_.empty()) {
                int total_phases = static_cast<int>(rc_inspect_order_.size()) * 2;
                int done_phases = static_cast<int>(rc_inspect_index_) * 2 + (rc_inspect_lock_phase_ ? 0 : 1);
                if (!rc_inspect_running_ && rc_inspect_index_ == 0 && inspect_attempts > 0) {
                    done_phases = total_phases;
                }
                float progress = total_phases > 0
                                     ? std::clamp(static_cast<float>(done_phases) / static_cast<float>(total_phases), 0.0f, 1.0f)
                                     : 0.0f;
                ImGui::ProgressBar(progress, ImVec2(-FLT_MIN, 0.0f));
                if (rc_inspect_running_ && rc_inspect_index_ < rc_inspect_order_.size()) {
                    ImGui::Text("当前步骤：%s -> %s", rc_inspect_order_[rc_inspect_index_].c_str(),
                                rc_inspect_lock_phase_ ? "锁定判定" : "解锁恢复");
                }
            }

            if (!rc_inspect_results_.empty()) {
                ImGuiTableFlags inspect_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
                if (ImGui::BeginTable("inspect_result_table", 5, inspect_flags, ImVec2(-FLT_MIN, 130.0f))) {
                    ImGui::TableSetupColumn("命令");
                    ImGui::TableSetupColumn("尝试");
                    ImGui::TableSetupColumn("通过");
                    ImGui::TableSetupColumn("超时");
                    ImGui::TableSetupColumn("最近延迟(s)");
                    ImGui::TableHeadersRow();
                    for (const auto& cmd : rc_inspect_order_) {
                        auto it = rc_inspect_results_.find(cmd);
                        if (it == rc_inspect_results_.end()) {
                            continue;
                        }
                        const RcInspectResult& result = it->second;
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(cmd.c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%d", result.attempts);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.45f, 1.0f), "%d", result.pass);
                        ImGui::TableSetColumnIndex(3);
                        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%d", result.timeout);
                        ImGui::TableSetColumnIndex(4);
                        if (result.last_latency_s >= 0.0) {
                            ImGui::Text("%.3f", result.last_latency_s);
                        } else {
                            ImGui::TextDisabled("-");
                        }
                    }
                    ImGui::EndTable();
                }
            }

            ImGui::SeparatorText("状态风险分析");
            struct RadarIssue {
                int weight = 0;
                ImVec4 color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
                std::string title;
                std::string detail;
            };
            std::vector<RadarIssue> radar_issues;
            auto push_issue = [&](int weight, const ImVec4& color, const std::string& title, const std::string& detail) {
                RadarIssue issue;
                issue.weight = weight;
                issue.color = color;
                issue.title = title;
                issue.detail = detail;
                radar_issues.push_back(std::move(issue));
            };

            if (!rc_virtual_joy_ready_) {
                push_issue(22, ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "命令通道异常", "无法发送 rc_virtual_joy 命令");
            }
            if (!omnilink_state_ready_) {
                push_issue(22, ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "状态通道异常", "无法接收 omnilink 状态消息");
            } else if (frame.state_msg_count == 0) {
                push_issue(12, ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "状态未上线", "状态话题尚未收到数据");
            } else if (!frame.state_data_fresh) {
                push_issue(14, ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "状态陈旧", "状态数据超时未更新");
            }

            auto conn_it = omnilink_state_map.find("TeleopDeviceConnection");
            if (conn_it != omnilink_state_map.end() && conn_it->second != 1) {
                std::ostringstream detail_ss;
                detail_ss << "TeleopDeviceConnection=" << conn_it->second;
                push_issue(20, ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "主臂设备连接异常", detail_ss.str());
            }
            auto robot_it = omnilink_state_map.find("RobotStates");
            if (robot_it != omnilink_state_map.end()) {
                if (robot_it->second == 3) {
                    push_issue(26, ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "机器人总状态错误", "RobotStates=3");
                } else if (robot_it->second == 2) {
                    push_issue(12, ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "机器人连接丢失", "RobotStates=2");
                }
            }

            for (const auto& cmd : config_.omnilink_bridge.rc_button_names) {
                if (rc_virtual_joy_command_[cmd] != 1) {
                    continue;
                }
                std::string state_name = FixCmdToStateName(cmd);
                auto state_it = omnilink_state_map.find(state_name);
                if (state_name.empty() || state_it == omnilink_state_map.end()) {
                    continue;
                }
                if (state_it->second != 0) {
                    std::ostringstream detail_ss;
                    detail_ss << cmd << " 已锁定但回传状态=" << state_it->second;
                    push_issue(8, ImVec4(1.0f, 0.78f, 0.2f, 1.0f), "锁定未生效", detail_ss.str());
                }
            }

            int radar_penalty = 0;
            for (const auto& issue : radar_issues) {
                radar_penalty += issue.weight;
            }
            int radar_score = std::clamp(100 - radar_penalty, 0, 100);
            ImVec4 radar_color = ImVec4(0.35f, 1.0f, 0.45f, 1.0f);
            const char* radar_level = "优秀";
            if (radar_score < 60) {
                radar_level = "高风险";
                radar_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
            } else if (radar_score < 85) {
                radar_level = "关注";
                radar_color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
            }

            if (radar_issues.empty()) {
                if (omnilink_clean_streak_start_s_ < 0.0) {
                    omnilink_clean_streak_start_s_ = frame.now_sec;
                }
                omnilink_best_clean_streak_s_ =
                    std::max(omnilink_best_clean_streak_s_, frame.now_sec - omnilink_clean_streak_start_s_);
            } else {
                if (omnilink_clean_streak_start_s_ >= 0.0) {
                    omnilink_best_clean_streak_s_ =
                        std::max(omnilink_best_clean_streak_s_, frame.now_sec - omnilink_clean_streak_start_s_);
                }
                omnilink_clean_streak_start_s_ = -1.0;
            }

            double clean_streak_s = omnilink_clean_streak_start_s_ >= 0.0 ? (frame.now_sec - omnilink_clean_streak_start_s_) : 0.0;
            ImGui::Text("雷达评分：%d / 100", radar_score);
            ImGui::SameLine();
            ImGui::TextColored(radar_color, "%s", radar_level);
            ImGui::Text("稳定连击：%.1f s（最佳 %.1f s）", clean_streak_s, omnilink_best_clean_streak_s_);
            ImGui::SameLine();
            if (ImGui::SmallButton("重置最佳连击")) {
                omnilink_best_clean_streak_s_ = 0.0;
                omnilink_clean_streak_start_s_ = radar_issues.empty() ? frame.now_sec : -1.0;
            }

            if (radar_issues.empty()) {
                ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.45f, 1.0f), "当前无异常，继续保持。");
            } else {
                ImGuiTableFlags radar_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
                if (ImGui::BeginTable("omnilink_radar_table", 3, radar_flags, ImVec2(-FLT_MIN, 120.0f))) {
                    ImGui::TableSetupColumn("类型");
                    ImGui::TableSetupColumn("详情");
                    ImGui::TableSetupColumn("权重");
                    ImGui::TableHeadersRow();
                    for (const auto& issue : radar_issues) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextColored(issue.color, "%s", issue.title.c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(issue.detail.c_str());
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%d", issue.weight);
                    }
                    ImGui::EndTable();
                }
            }

            ImGui::SeparatorText("手动命令控制");
            ImGui::SliderFloat("生效超时阈值(s)", &rc_virtual_joy_effect_timeout_s_, 0.2f, 5.0f, "%.1f");

            bool changed = false;
            std::vector<std::string> changed_cmd_names;
            bool manual_cmd_disabled = rc_inspect_running_ || (error_monitor_enabled && frame.error_active_count > 0);
            ImGui::BeginDisabled(manual_cmd_disabled);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.42f, 0.18f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.56f, 0.24f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.34f, 0.15f, 0.15f, 1.0f));
            if (ImGui::Button("全锁定")) {
                for (const auto& name : config_.omnilink_bridge.rc_button_names) {
                    rc_virtual_joy_command_[name] = 1;
                    rc_virtual_joy_changed_at_s_[name] = frame.now_sec;
                    changed_cmd_names.push_back(name);
                }
                changed = true;
            }
            ImGui::PopStyleColor(3);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.38f, 0.23f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.21f, 0.50f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.14f, 0.31f, 0.19f, 1.0f));
            if (ImGui::Button("全解锁")) {
                for (const auto& name : config_.omnilink_bridge.rc_button_names) {
                    rc_virtual_joy_command_[name] = 0;
                    rc_virtual_joy_changed_at_s_[name] = frame.now_sec;
                    changed_cmd_names.push_back(name);
                }
                changed = true;
            }
            ImGui::PopStyleColor(3);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.30f, 0.45f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.40f, 0.58f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.14f, 0.24f, 0.36f, 1.0f));
            if (ImGui::Button("仅双臂解锁")) {
                for (const auto& name : config_.omnilink_bridge.rc_button_names) {
                    int value = (name == "left_arm_fix" || name == "right_arm_fix") ? 0 : 1;
                    if (rc_virtual_joy_command_[name] != value) {
                        rc_virtual_joy_command_[name] = value;
                        rc_virtual_joy_changed_at_s_[name] = frame.now_sec;
                        changed_cmd_names.push_back(name);
                        changed = true;
                    }
                }
            }
            ImGui::PopStyleColor(3);
            ImGui::SameLine();
            if (ImGui::Button("立即发送")) {
                publishRcVirtualJoyCommand("ui.manual_send", "manual");
            }

            int lock_ok_count = 0;
            int lock_wait_count = 0;
            int lock_timeout_count = 0;
            int lock_unknown_count = 0;

            ImGuiTableFlags cmd_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
            if (ImGui::BeginTable("omnilink_cmd_table", 3, cmd_flags, ImVec2(-FLT_MIN, 180.0f))) {
                ImGui::TableSetupColumn("命令");
                ImGui::TableSetupColumn("锁定");
                ImGui::TableSetupColumn("回传一致性");
                ImGui::TableHeadersRow();
                for (const auto& name : config_.omnilink_bridge.rc_button_names) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(name.c_str());

                    ImGui::TableSetColumnIndex(1);
                    bool lock_on = rc_virtual_joy_command_[name] == 1;
                    std::string id = "##rc_fix_" + name;
                    if (ImGui::Checkbox(id.c_str(), &lock_on)) {
                        rc_virtual_joy_command_[name] = lock_on ? 1 : 0;
                        rc_virtual_joy_changed_at_s_[name] = frame.now_sec;
                        changed_cmd_names.push_back(name);
                        changed = true;
                    }

                    ImGui::TableSetColumnIndex(2);
                    std::string state_name = FixCmdToStateName(name);
                    auto state_it = omnilink_state_map.find(state_name);
                    if (state_name.empty() || state_it == omnilink_state_map.end()) {
                        ImGui::TextDisabled("未知");
                        if (lock_on) {
                            lock_unknown_count++;
                        }
                    } else if (!lock_on) {
                        ImGui::TextDisabled("未锁定，不校验");
                    } else if (state_it->second == 0) {
                        ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.45f, 1.0f), "已生效");
                        lock_ok_count++;
                        rc_virtual_joy_changed_at_s_.erase(name);
                    } else {
                        double dt = -1.0;
                        auto changed_it = rc_virtual_joy_changed_at_s_.find(name);
                        if (changed_it != rc_virtual_joy_changed_at_s_.end()) {
                            dt = frame.now_sec - changed_it->second;
                        }
                        if (dt >= rc_virtual_joy_effect_timeout_s_) {
                            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "超时(%d, %.2fs)", state_it->second, dt);
                            lock_timeout_count++;
                        } else {
                            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "待生效(%d)", state_it->second);
                            lock_wait_count++;
                        }
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndDisabled();

            ImGui::Text("锁定一致性：已生效 %d  待生效 %d  超时 %d  未知 %d",
                        lock_ok_count, lock_wait_count, lock_timeout_count, lock_unknown_count);

            if (rc_inspect_running_) {
                ImGui::TextDisabled("自动巡检进行中，手动命令编辑已临时禁用。");
            } else if (error_monitor_enabled && frame.error_active_count > 0) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "机器人处于错误状态，手动命令发送已禁用。");
            } else if (changed) {
                std::ostringstream note_ss;
                note_ss << "changed=" << changed_cmd_names.size();
                if (!changed_cmd_names.empty()) {
                    note_ss << " [" << changed_cmd_names.front();
                    if (changed_cmd_names.size() > 1) {
                        note_ss << ", ...";
                    }
                    note_ss << "]";
                }
                publishRcVirtualJoyCommand("ui.changed", note_ss.str());
            }

            ImGuiTableFlags state_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
            if (ImGui::BeginTable("omnilink_state_table", 4, state_flags, ImVec2(-FLT_MIN, 170.0f))) {
                ImGui::TableSetupColumn("状态项");
                ImGui::TableSetupColumn("原始名");
                ImGui::TableSetupColumn("数值");
                ImGui::TableSetupColumn("解释");
                ImGui::TableHeadersRow();
                for (const auto& [state_name, value] : omnilink_state_map) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(OmnilinkStateNameZh(state_name));
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextDisabled("%s", state_name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d", value);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextColored(OmnilinkStateColor(state_name, value), "%s", OmnilinkStateValueZh(state_name, value));
                }
                ImGui::EndTable();
            }

            ImGui::Separator();
            ImGui::Text("命令历史");
            if (rc_virtual_joy_logs_.empty()) {
                ImGui::TextDisabled("暂无历史发送记录");
            } else {
                ImGuiTableFlags log_flags =
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
                if (ImGui::BeginTable("omnilink_cmd_log_table", 5, log_flags, ImVec2(-FLT_MIN, 140.0f))) {
                    ImGui::TableSetupColumn("时间(s)");
                    ImGui::TableSetupColumn("来源");
                    ImGui::TableSetupColumn("结果");
                    ImGui::TableSetupColumn("锁定数");
                    ImGui::TableSetupColumn("备注");
                    ImGui::TableHeadersRow();
                    for (const auto& log : rc_virtual_joy_logs_) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%.2f", log.time_s);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(log.source.c_str());
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextColored(log.ok ? ImVec4(0.35f, 1.0f, 0.45f, 1.0f) : ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                           "%s", log.ok ? "OK" : "FAIL");
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%d", log.active_locks);
                        ImGui::TableSetColumnIndex(4);
                        ImGui::TextUnformatted(log.note.c_str());
                    }
                    ImGui::EndTable();
                }
            }
        }
    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool joy_section_open = ImGui::CollapsingHeader("主臂手柄信息###section_joy", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (joy_section_open) {
    ImGui::Text("Joy Topic：");
    ImGui::TextWrapped("%s", config_.joy.topic.c_str());

    if (!joy_ready_) {
        ImGui::Text("Joy 状态：");
        ImGui::SameLine();
        DrawStatusBadge("joy_init", "初始化失败", ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
    } else if (frame.joy_msg_count == 0) {
        ImGui::Text("Joy 状态：");
        ImGui::SameLine();
        DrawStatusBadge("joy_wait", "等待数据", ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    } else if (!frame.joy_data_fresh) {
        ImGui::Text("Joy 状态：");
        ImGui::SameLine();
        DrawStatusBadge("joy_stale", "数据陈旧", ImVec4(1.0f, 0.7f, 0.2f, 1.0f));
        ImGui::SameLine();
        ImGui::TextDisabled("(%.3f s 前)", frame.joy_data_age);
    } else {
        ImGui::Text("Joy 状态：");
        ImGui::SameLine();
        DrawStatusBadge("joy_ok", "接收中", ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
    }
    ImGui::Text("Joy 消息计数：%llu", static_cast<unsigned long long>(frame.joy_msg_count));

    std::map<std::string, int> left_buttons;
    std::map<std::string, int> right_buttons;
    std::map<std::string, double> left_axes;
    std::map<std::string, double> right_axes;
    for (const auto& button : latest_joy_buttons_) {
        if (IsLeftHandleKey(button.name)) {
            left_buttons[button.name] = button.status;
        } else if (IsRightHandleKey(button.name)) {
            right_buttons[button.name] = button.status;
        }
    }
    for (const auto& axis : latest_joy_axes_) {
        if (!axis.has_value) {
            continue;
        }
        if (IsLeftHandleKey(axis.name)) {
            left_axes[axis.name] = axis.value;
        } else if (IsRightHandleKey(axis.name)) {
            right_axes[axis.name] = axis.value;
        }
    }

    auto draw_handle_panel = [&](const char* title, const std::map<std::string, double>& axes,
                                 const std::map<std::string, int>& buttons, bool is_left) {
        std::vector<std::string> x_names;
        std::vector<std::string> y_names;
        std::vector<std::string> trig_names;
        if (is_left) {
            x_names = {"poleL_x", "left_x", "left_stick_x"};
            y_names = {"poleL_y", "left_y", "left_stick_y"};
            trig_names = {"triggerL_x", "left_trigger", "left_trigger_x"};
        } else {
            x_names = {"poleR_x", "right_x", "right_stick_x"};
            y_names = {"poleR_y", "right_y", "right_stick_y"};
            trig_names = {"triggerR_x", "right_trigger", "right_trigger_x"};
        }

        bool has_x = false;
        bool has_y = false;
        bool has_t = false;
        float raw_x = static_cast<float>(GetAxisValue(axes, x_names, &has_x));
        float raw_y = static_cast<float>(GetAxisValue(axes, y_names, &has_y));
        float trg = static_cast<float>(GetAxisValue(axes, trig_names, &has_t));

        // 与 omnilink updateLeftAxis/updateRightAxis 保持一致，并将显示Y轴取反（左右方向反向）。
        // lr = (50 - y) / 50 * sign, fb = (x - 50) / 50 * sign
        float sign = is_left ? -1.0f : 1.0f;
        float js_y = -((50.0f - raw_y) / 50.0f) * sign;
        float js_x = ((raw_x - 50.0f) / 50.0f) * sign;

        ImGui::TextUnformatted(title);
        DrawJoyStickPad(is_left ? "##joy_pad_left" : "##joy_pad_right", js_y, js_x, ImVec2(130, 130));
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::Text("摇杆 X(上下): % .3f", js_x);
        ImGui::Text("摇杆 Y(左右): % .3f", js_y);
        if (has_t) {
            ImGui::Text("扳机: % .3f", trg);
        } else {
            ImGui::TextDisabled("扳机: N/A");
        }
        if (!has_x || !has_y) {
            ImGui::TextDisabled("提示：未识别到完整摇杆轴");
        }
        ImGui::EndGroup();

        ImGuiTableFlags btn_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
        if (ImGui::BeginTable(is_left ? "left_handle_btn_table" : "right_handle_btn_table", 2, btn_flags, ImVec2(-FLT_MIN, 170.0f))) {
            ImGui::TableSetupColumn("按键");
            ImGui::TableSetupColumn("状态");
            ImGui::TableHeadersRow();
            if (buttons.empty()) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("无数据");
            } else {
                for (const auto& [name, status] : buttons) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    std::string alias = JoyButtonAliasName(name);
                    ImGui::TextUnformatted(alias.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextColored(JoyButtonStateColor(status), "%s (%d)", JoyButtonStateText(status), status);
                }
            }
            ImGui::EndTable();
        }
    };

    ImGuiTableFlags handle_table_flags = ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("handle_table", 2, handle_table_flags, ImVec2(-FLT_MIN, 0.0f))) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        draw_handle_panel("左手柄", left_axes, left_buttons, true);
        ImGui::TableSetColumnIndex(1);
        draw_handle_panel("右手柄", right_axes, right_buttons, false);
        ImGui::EndTable();
    }

    ImGui::Separator();
    ImGui::Text("按键历史");
    ImGui::SameLine();
    if (ImGui::SmallButton("清空按键历史")) {
        joy_button_logs_.clear();
    }
    if (joy_button_logs_.empty()) {
        ImGui::TextDisabled("暂无按键历史事件。");
    } else {
        ImGuiTableFlags hist_flags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
        if (ImGui::BeginTable("joy_button_history_table", 4, hist_flags, ImVec2(-FLT_MIN, 160.0f))) {
            ImGui::TableSetupColumn("时间(s)");
            ImGui::TableSetupColumn("手柄");
            ImGui::TableSetupColumn("按键");
            ImGui::TableSetupColumn("状态");
            ImGui::TableHeadersRow();
            for (const auto& item : joy_button_logs_) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.2f", item.time_s);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(item.hand.c_str());
                ImGui::TableSetColumnIndex(2);
                std::string alias = JoyButtonAliasName(item.name);
                ImGui::TextUnformatted(alias.c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextColored(JoyButtonStateColor(item.status), "%s (%d)", JoyButtonStateText(item.status), item.status);
            }
            ImGui::EndTable();
        }
    }

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool health_section_open = ImGui::CollapsingHeader("健康总览###section_health", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (health_section_open) {
    ImGui::Text("输入频率：%.1f Hz", input_rate_hz_);
    ImGui::Text("数据时延：%.3f s", frame.data_age >= 0.0 ? frame.data_age : -1.0);

    int health_penalty =
        frame.invalid_count * 25 + frame.out_range_count * 15 + frame.no_match_count * 10 + (!frame.data_fresh ? 20 : 0);
    int health_score = std::clamp(100 - health_penalty, 0, 100);
    const char* health_state = "健康";
    ImVec4 health_state_color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
    if (health_score < 60) {
        health_state = "严重";
        health_state_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
    } else if (health_score < 85) {
        health_state = "告警";
        health_state_color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
    }
    ImGui::Text("健康评分：%d / 100", health_score);
    ImGui::TextColored(health_state_color, "状态：%s", health_state);

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool session_section_open = ImGui::CollapsingHeader("会话工具###section_session", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (session_section_open) {
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
        baseline_capture_s = frame.now_sec;
    }
    ImGui::SameLine();
    if (ImGui::Button("清除基线")) {
        baseline_position_rad_.clear();
        baseline_ready_ = false;
    }

    if (!recording_) {
        if (ImGui::Button("开始录制 CSV")) {
            startRecording(frame.now_sec);
        }
    } else {
        if (ImGui::Button("停止录制 CSV")) {
            stopRecording();
        }
    }

    if (baseline_ready_) {
        ImGui::Text("基线关节数：%d（%.1f s 前抓取）", static_cast<int>(baseline_position_rad_.size()),
                    frame.now_sec - baseline_capture_s);
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

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool joint_diag_section_open = ImGui::CollapsingHeader("关节诊断###section_joint_diag", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (joint_diag_section_open) {
    ImGui::Text("显示：%d  无效：%d  超限：%d  无模型匹配：%d", frame.shown_count, frame.invalid_count,
                frame.out_range_count, frame.no_match_count);

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
            auto diag_it = frame.diag_map.find(composeJointKey(sample));
            if (diag_it != frame.diag_map.end()) {
                diag = diag_it->second;
            } else {
                diag = evaluateJoint(sample);
            }

            const char* health_text = "正常";
            ImVec4 health_color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
            if (diag.invalid) {
                health_text = "无效";
                health_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
            } else if (diag.no_match) {
                health_text = "无URDF匹配";
                health_color = ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
            } else if (diag.out_of_range) {
                health_text = "超限";
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
            if (baseline_ready_ && baseline_it != baseline_position_rad_.end() && sample.has_position &&
                std::isfinite(sample.position)) {
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

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool waveform_section_open = ImGui::CollapsingHeader("关节波形###section_waveform", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (waveform_section_open) {

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
        if (std::find(selectable_joints.begin(), selectable_joints.end(), selected_wave_joint_) == selectable_joints.end()) {
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
        const char* selected_joint_preview = selected_wave_joint_.empty() ? "(none)" : selected_wave_joint_.c_str();
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
            std::string waveform_key = selected_wave_group_ + "/" + selected_wave_joint_ + "/" + WaveMetricKeySuffix(selected_wave_metric_);
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
                ImGui::Text("当前值：%.4f  最小：%.4f  最大：%.4f  样本数：%d", plot_values.back(), min_value, max_value,
                            static_cast<int>(plot_values.size()));
            }
        }
    }

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool alarm_section_open = ImGui::CollapsingHeader("告警中心###section_alarm", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (alarm_section_open) {
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
    std::sort(alarm_rows.begin(), alarm_rows.end(), [](const auto& a, const auto& b) {
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

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool manual_section_open = ImGui::CollapsingHeader("手动模式（本地调试）###section_manual", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (manual_section_open) {
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

    }

    ImGui::Spacing();
    PushSectionHeaderStyle();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    bool camera_help_section_open = ImGui::CollapsingHeader("相机操作说明###section_camera_help", ImGuiTreeNodeFlags_DefaultOpen);
    PopSectionHeaderStyle();
    if (camera_help_section_open) {
    ImGui::Text("左键拖动：旋转（RViz Orbit）");
    ImGui::Text("中键拖动 或 Shift+左键：平移");
    ImGui::Text("右键拖动：Dolly 缩放");
    ImGui::Text("滚轮：缩放");
    }
    ImGui::End();
}


}  // namespace omnilink::teleop_viewer
