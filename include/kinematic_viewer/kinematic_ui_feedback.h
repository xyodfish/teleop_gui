#pragma once

#include "imgui.h"

#include <string>
#include <vector>

namespace kinematic_viewer {

enum class UiSemanticLevel {
    Info = 0,
    Success = 1,
    Warning = 2,
    Error = 3,
};

class KinematicUiFeedback {
   public:
    void Push(UiSemanticLevel level, const std::string& message, double nowSec, double durationSec = 2.8);
    void RenderToasts(double nowSec, float anchorRightX = 0.0f, float anchorTopY = 0.0f);

    static ImVec4 SemanticColor(UiSemanticLevel level);
    static void RenderStatusChip(const char* label, UiSemanticLevel level);

   private:
    struct ToastItem {
        UiSemanticLevel level = UiSemanticLevel::Info;
        std::string message;
        double expireSec = 0.0;
    };

    std::vector<ToastItem> toasts_;
};

}  // namespace kinematic_viewer
