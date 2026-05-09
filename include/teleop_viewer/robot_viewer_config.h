#pragma once

#include "teleop_viewer/config_types.h"

#include <string>
#include <vector>

namespace omnilink::teleop_viewer {

struct SensorConfig {
    std::string topic = "singorix_omnilink/scaled_device_robot_data";
    std::string node_name = "singorix_teleop_gui_sensor_monitor";
};

struct JoyConfig {
    std::string topic = "singorix_omnilink/joy";
};

struct OmnilinkBridgeConfig {
    bool enable = true;
    std::string rc_virtual_joy_topic = "omnilink_comm/rc_common_cmd";
    std::string state_topic = "singorix_omnilink/states";
    std::string wbc_info_topic = "singorix/wbcs/wbc_info";
    std::string error_topic = "singorix/wbcs/error";
    bool enable_wbc_monitor = true;
    bool enable_error_monitor = true;
    bool auto_lock_on_critical_fault = false;
    std::vector<std::string> rc_button_names = {"fix_height",      "head_angle_fix",   "chassis_fix",   "left_arm_fix",
                                                "right_arm_fix",   "left_gripper_fix", "right_gripper_fix"};
    double command_repeat_interval_sec = 0.3;
    double wbc_norm_warn = 0.2;
    double wbc_norm_danger = 0.5;
};

struct RobotViewerConfig {
    WindowConfig window;
    RobotConfig robot;
    SensorConfig sensor;
    JoyConfig joy;
    OmnilinkBridgeConfig omnilink_bridge;
    CameraConfig camera;
    UiConfig ui;
    ViewerIkConfig ik;

    static RobotViewerConfig LoadFromFile(const std::string& yaml_path, bool* loaded_ok = nullptr);
};

}  // namespace omnilink::teleop_viewer
