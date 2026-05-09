#pragma once

#include "teleop_viewer/config_types.h"

#include <string>

namespace omnilink::teleop_viewer {

struct KinematicRosConfig {
    bool enable = true;
};

struct KinematicViewerConfig {
    WindowConfig window;
    RobotConfig robot;
    CameraConfig camera;
    UiConfig ui;
    ViewerIkConfig ik;
    KinematicRosConfig ros;

    static KinematicViewerConfig LoadFromFile(const std::string& yaml_path, bool* loaded_ok = nullptr);
};

}  // namespace omnilink::teleop_viewer
