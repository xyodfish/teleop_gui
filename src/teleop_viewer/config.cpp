#include "teleop_viewer/config.h"

#include <yaml-cpp/yaml.h>

#include <iostream>

namespace omnilink::teleop_viewer {
namespace {

template <typename T>
void ReadScalar(const YAML::Node& node, const char* key, T& out) {
    if (node && node[key]) {
        out = node[key].as<T>();
    }
}

void ReadVec3(const YAML::Node& node, const char* key, glm::vec3& out) {
    if (!node || !node[key] || !node[key].IsSequence() || node[key].size() < 3) {
        return;
    }
    out.x = node[key][0].as<float>();
    out.y = node[key][1].as<float>();
    out.z = node[key][2].as<float>();
}

}  // namespace

ViewerConfig ViewerConfig::LoadFromFile(const std::string& yaml_path, bool* loaded_ok) {
    ViewerConfig cfg;
    bool ok = false;

    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        ReadScalar(root["window"], "width", cfg.window.width);
        ReadScalar(root["window"], "height", cfg.window.height);
        ReadScalar(root["window"], "title", cfg.window.title);

        ReadScalar(root["robot"], "urdf_path", cfg.robot.urdf_path);

        ReadScalar(root["sensor"], "topic", cfg.sensor.topic);
        ReadScalar(root["sensor"], "node_name", cfg.sensor.node_name);

        ReadScalar(root["camera"], "distance", cfg.camera.distance);
        ReadScalar(root["camera"], "yaw", cfg.camera.yaw);
        ReadScalar(root["camera"], "pitch", cfg.camera.pitch);
        ReadVec3(root["camera"], "target", cfg.camera.target);
        ReadScalar(root["camera"], "rotate_speed", cfg.camera.rotate_speed);
        ReadScalar(root["camera"], "zoom_scale", cfg.camera.zoom_scale);
        ReadScalar(root["camera"], "dolly_scale", cfg.camera.dolly_scale);
        ReadScalar(root["camera"], "pan_scale", cfg.camera.pan_scale);
        ReadScalar(root["camera"], "min_distance", cfg.camera.min_distance);
        ReadScalar(root["camera"], "max_distance", cfg.camera.max_distance);

        ReadScalar(root["ui"], "only_show_master_arm_groups", cfg.ui.only_show_master_arm_groups);
        ReadScalar(root["ui"], "fix_base_like_mujoco", cfg.ui.fix_base_like_mujoco);
        ReadScalar(root["ui"], "sidebar_collapsed_default", cfg.ui.sidebar_collapsed_default);
        ReadScalar(root["ui"], "side_panel_width", cfg.ui.side_panel_width);
        ReadScalar(root["ui"], "collapsed_sidebar_width", cfg.ui.collapsed_sidebar_width);
        ReadScalar(root["ui"], "sidebar_width_drag_speed", cfg.ui.sidebar_width_drag_speed);
        ReadScalar(root["ui"], "stale_timeout_seconds", cfg.ui.stale_timeout_seconds);
        ReadScalar(root["ui"], "out_of_range_margin", cfg.ui.out_of_range_margin);
        ReadScalar(root["ui"], "waveform_history_size", cfg.ui.waveform_history_size);
        ReadScalar(root["ui"], "waveform_plot_height", cfg.ui.waveform_plot_height);
        ReadScalar(root["ui"], "alarm_trigger_frames", cfg.ui.alarm_trigger_frames);
        ReadScalar(root["ui"], "alarm_show_only_active_default", cfg.ui.alarm_show_only_active_default);
        ReadScalar(root["ui"], "baseline_warn_delta_deg", cfg.ui.baseline_warn_delta_deg);
        ReadScalar(root["ui"], "record_output_dir", cfg.ui.record_output_dir);
        ReadScalar(root["ui"], "auto_start_recording", cfg.ui.auto_start_recording);
        ReadScalar(root["ui"], "cjk_font_path", cfg.ui.cjk_font_path);
        ReadScalar(root["ui"], "cjk_font_size", cfg.ui.cjk_font_size);

        ok = true;
    } catch (const std::exception& e) {
        std::cerr << "[ViewerConfig] Load failed for: " << yaml_path << ", reason: " << e.what()
                  << ". Fallback to defaults." << std::endl;
    }

    if (loaded_ok) {
        *loaded_ok = ok;
    }
    return cfg;
}

}  // namespace omnilink::teleop_viewer
