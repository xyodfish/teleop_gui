#include "kinematic_viewer/kinematic_ui_feedback.h"

#include <algorithm>

namespace kinematic_viewer {
namespace kinematic_ui_feedback_internal {

ImU32 ColorU32(const ImVec4& c) { return ImGui::ColorConvertFloat4ToU32(c); }

}  // namespace kinematic_ui_feedback_internal

void KinematicUiFeedback::Push(UiSemanticLevel level, const std::string& message, double nowSec, double durationSec) {
    if (message.empty()) {
        return;
    }
    ToastItem item;
    item.level     = level;
    item.message   = message;
    item.expireSec = nowSec + std::max(0.6, durationSec);
    toasts_.push_back(std::move(item));
    if (toasts_.size() > 6) {
        toasts_.erase(toasts_.begin(), toasts_.begin() + static_cast<long>(toasts_.size() - 6));
    }
}

ImVec4 KinematicUiFeedback::SemanticColor(UiSemanticLevel level) {
    if (level == UiSemanticLevel::Success) {
        return ImVec4(0.34f, 0.82f, 0.47f, 1.0f);
    }
    if (level == UiSemanticLevel::Warning) {
        return ImVec4(0.95f, 0.75f, 0.22f, 1.0f);
    }
    if (level == UiSemanticLevel::Error) {
        return ImVec4(0.95f, 0.38f, 0.38f, 1.0f);
    }
    return ImVec4(0.45f, 0.72f, 0.97f, 1.0f);
}

void KinematicUiFeedback::RenderStatusChip(const char* label, UiSemanticLevel level) {
    if (label == nullptr) {
        return;
    }
    const ImVec4 color = SemanticColor(level);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(color.x, color.y, color.z, 0.16f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(color.x, color.y, color.z, 0.16f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(color.x, color.y, color.z, 0.16f));
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 999.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 4.0f));
    ImGui::Button(label);
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(4);
}

void KinematicUiFeedback::RenderToasts(double nowSec, float anchorRightX, float anchorTopY) {
    toasts_.erase(std::remove_if(toasts_.begin(), toasts_.end(), [&](const ToastItem& item) { return item.expireSec <= nowSec; }), toasts_.end());
    if (toasts_.empty()) {
        return;
    }

    const float right = (anchorRightX > 1.0f) ? anchorRightX : ImGui::GetMainViewport()->Pos.x + ImGui::GetMainViewport()->Size.x;
    const float top = (anchorTopY > 0.0f) ? anchorTopY : (ImGui::GetMainViewport()->Pos.y + 14.0f);

    for (size_t i = 0; i < toasts_.size(); ++i) {
        const ToastItem& item = toasts_[toasts_.size() - 1 - i];
        const ImVec4 semantic = SemanticColor(item.level);
        const ImVec2 pos(right - 14.0f, top + static_cast<float>(i) * 46.0f);
        ImGui::SetNextWindowPos(pos, ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.92f);

        std::string windowName = "##kinematic_toast_" + std::to_string(i);
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                                 ImGuiWindowFlags_NoInputs;
        if (ImGui::Begin(windowName.c_str(), nullptr, flags)) {
            const ImVec2 pMin = ImGui::GetWindowPos();
            const ImVec2 pMax = ImVec2(pMin.x + ImGui::GetWindowSize().x, pMin.y + ImGui::GetWindowSize().y);
            ImGui::GetWindowDrawList()->AddRectFilled(pMin, ImVec2(pMin.x + 3.0f, pMax.y),
                                                      kinematic_ui_feedback_internal::ColorU32(semantic), 2.0f);
            ImGui::TextColored(semantic, "%s", item.level == UiSemanticLevel::Error ? "错误" :
                                               item.level == UiSemanticLevel::Warning ? "告警" :
                                               item.level == UiSemanticLevel::Success ? "完成" : "提示");
            ImGui::SameLine();
            ImGui::TextUnformatted(item.message.c_str());
        }
        ImGui::End();
    }
}

}  // namespace kinematic_viewer
