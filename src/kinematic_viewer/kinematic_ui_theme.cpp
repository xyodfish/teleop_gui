#include "kinematic_viewer/kinematic_ui_theme.h"

#include "imgui.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace kinematic_viewer {

static bool FileExists(const std::string& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec);
}

void SetupKinematicViewerFonts(const KinematicViewerConfig& cfg) {
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
        if (!FileExists(path)) {
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

void ApplyKinematicUiStyle() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 10.0f;
    style.ChildRounding     = 8.0f;
    style.PopupRounding     = 8.0f;
    style.FrameRounding     = 8.0f;
    style.GrabRounding      = 8.0f;
    style.ScrollbarRounding = 10.0f;
    style.TabRounding       = 8.0f;
    style.WindowPadding     = ImVec2(12.0f, 10.0f);
    style.FramePadding      = ImVec2(10.0f, 7.0f);
    style.ItemSpacing       = ImVec2(9.0f, 8.0f);
    style.ItemInnerSpacing  = ImVec2(8.0f, 6.0f);
    style.IndentSpacing     = 18.0f;
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 1.0f;
    style.ScrollbarSize     = 15.0f;
    style.GrabMinSize       = 12.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.11f, 0.13f, 0.17f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.27f, 0.33f, 0.40f, 0.90f);
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_Text]                 = ImVec4(0.92f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.58f, 0.64f, 0.71f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.13f, 0.18f, 0.24f, 1.00f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.14f, 0.18f, 0.24f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.19f, 0.28f, 0.40f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.22f, 0.35f, 0.50f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.18f, 0.30f, 0.44f, 0.85f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.23f, 0.41f, 0.60f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.28f, 0.49f, 0.70f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.30f, 0.44f, 0.70f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.23f, 0.41f, 0.60f, 0.88f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.28f, 0.49f, 0.70f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.41f, 0.74f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.37f, 0.69f, 0.97f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.46f, 0.79f, 1.00f, 1.00f);
    colors[ImGuiCol_ResizeGrip]           = ImVec4(0.36f, 0.62f, 0.87f, 0.35f);
    colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.41f, 0.74f, 1.00f, 0.75f);
    colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.47f, 0.81f, 1.00f, 1.00f);
    colors[ImGuiCol_Separator]            = ImVec4(0.27f, 0.33f, 0.40f, 0.95f);
    colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.13f, 0.19f, 0.27f, 1.00f);
    colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.29f, 0.36f, 0.44f, 1.00f);
    colors[ImGuiCol_TableBorderLight]     = ImVec4(0.20f, 0.26f, 0.33f, 1.00f);
    colors[ImGuiCol_TableRowBg]           = ImVec4(0.10f, 0.12f, 0.16f, 0.45f);
    colors[ImGuiCol_TableRowBgAlt]        = ImVec4(0.12f, 0.15f, 0.20f, 0.65f);
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.30f, 0.39f, 0.48f, 0.95f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.39f, 0.51f, 0.62f, 0.95f);
    colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.49f, 0.63f, 0.75f, 1.00f);
}

}  // namespace kinematic_viewer
