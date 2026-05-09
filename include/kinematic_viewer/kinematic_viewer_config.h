#pragma once

#include "teleop_viewer/config_types.h"

#include <string>

namespace kinematic_viewer {

using CameraConfig        = omnilink::teleop_viewer::CameraConfig;
using RobotConfig         = omnilink::teleop_viewer::RobotConfig;
using UiConfig            = omnilink::teleop_viewer::UiConfig;
using ViewerIkConfig      = omnilink::teleop_viewer::ViewerIkConfig;
using ViewerIkChainConfig = omnilink::teleop_viewer::ViewerIkChainConfig;
using WindowConfig        = omnilink::teleop_viewer::WindowConfig;

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

}  // namespace kinematic_viewer
