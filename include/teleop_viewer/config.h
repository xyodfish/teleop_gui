#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace omnilink::teleop_viewer {

struct WindowConfig {
    int width  = 1280;
    int height = 720;
    std::string title = "遥操作机器人监控界面";
};

struct RobotConfig {
    std::string urdf_path =
        "/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/galbot_description/galbot_one_golf_description/"
        "galbot_one_golf.urdf";
};

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
    std::vector<std::string> rc_button_names = {"fix_height",      "head_angle_fix",   "chassis_fix",   "left_arm_fix",
                                                "right_arm_fix",   "left_gripper_fix", "right_gripper_fix"};
    double command_repeat_interval_sec = 0.3;
};

struct CameraConfig {
    float distance = 3.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);

    float rotate_speed = 0.005f;
    float zoom_scale = 0.1f;
    float dolly_scale = 0.02f;
    float pan_scale = 0.0015f;
    float min_distance = 0.2f;
    float max_distance = 20.0f;
};

struct UiConfig {
    bool only_show_master_arm_groups = true;
    bool fix_base_like_mujoco = true;
    bool sidebar_collapsed_default = false;

    int side_panel_width = 560;
    int collapsed_sidebar_width = 34;
    float sidebar_width_drag_speed = 0.2f;

    double stale_timeout_seconds = 0.5;
    double out_of_range_margin = 0.01;

    int waveform_history_size = 600;
    float waveform_plot_height = 180.0f;

    int alarm_trigger_frames = 5;
    bool alarm_show_only_active_default = true;
    float baseline_warn_delta_deg = 15.0f;

    std::string record_output_dir = "logs";
    bool auto_start_recording = false;

    std::string cjk_font_path = "";
    float cjk_font_size = 18.0f;
};

struct ViewerConfig {
    WindowConfig window;
    RobotConfig robot;
    SensorConfig sensor;
    JoyConfig joy;
    OmnilinkBridgeConfig omnilink_bridge;
    CameraConfig camera;
    UiConfig ui;

    static ViewerConfig LoadFromFile(const std::string& yaml_path, bool* loaded_ok = nullptr);
};

}  // namespace omnilink::teleop_viewer
