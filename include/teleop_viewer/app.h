#pragma once

#include <chrono>
#include <deque>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "teleop_viewer/config.h"
#include "teleop_viewer/scene.h"
#include "teleop_viewer/sensor_subscriber.h"

struct GLFWwindow;

namespace omnilink::teleop_viewer {

class RobotViewerApp {
   public:
    explicit RobotViewerApp(ViewerConfig config);
    ~RobotViewerApp();

    int run();

   private:
    enum class CameraDragMode { None, Rotate, Pan, Dolly };
    struct JointDiagState {
        bool invalid      = false;
        bool no_match     = false;
        bool out_of_range = false;
        bool has_info     = false;
        double min_angle  = 0.0;
        double max_angle  = 0.0;
    };
    struct AlarmEntry {
        std::string group;
        std::string joint;
        std::string reason;
        int trigger_count   = 0;
        double first_seen_s = 0.0;
        double last_seen_s  = 0.0;
        bool active         = true;
        bool acknowledged   = false;
    };
    struct FrameLayout {
        int window_width        = 0;
        int window_height       = 0;
        int min_sidebar_width   = 0;
        int max_sidebar_width   = 0;
        int active_sidebar_width = 0;
        int render_width        = 0;
        int render_height       = 0;
    };
    struct FrameData {
        double now_sec      = 0.0;
        uint64_t msg_count  = 0;
        double data_age     = -1.0;
        bool data_fresh     = false;
        uint64_t joy_msg_count = 0;
        double joy_data_age    = -1.0;
        bool joy_data_fresh    = false;
        uint64_t state_msg_count = 0;
        double state_data_age    = -1.0;
        bool state_data_fresh    = false;
        uint64_t wbc_msg_count = 0;
        double wbc_data_age    = -1.0;
        bool wbc_data_fresh    = false;
        uint64_t error_msg_count = 0;
        double error_data_age    = -1.0;
        bool error_data_fresh    = false;
        int error_active_count   = 0;
        std::unordered_map<std::string, JointDiagState> diag_map;
        int invalid_count   = 0;
        int out_range_count = 0;
        int no_match_count  = 0;
        int shown_count     = 0;
    };
    struct RcCommandLogEntry {
        double time_s = 0.0;
        std::string source;
        bool ok = false;
        int active_locks = 0;
        std::string note;
    };
    struct RcInspectResult {
        int attempts = 0;
        int pass = 0;
        int timeout = 0;
        double last_latency_s = -1.0;
    };
    struct JoyButtonLogEntry {
        double time_s = 0.0;
        std::string hand;
        std::string name;
        int status = 0;
    };

    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    void onScroll(double yoffset);

    bool initWindow();
    bool initImGui();
    bool initShader();
    bool initScene();
    void shutdown();

    void processCameraInput(bool imgui_want_capture_mouse);
    double nowSec() const;
    std::string composeJointKey(const SensorJointSample& sample) const;
    JointDiagState evaluateJoint(const SensorJointSample& sample) const;
    void updateAlarmForJoint(const SensorJointSample& sample, const JointDiagState& diag, double now_sec);
    void updateInputRate(uint64_t msg_count, double now_sec);
    void startRecording(double now_sec);
    void stopRecording();
    void recordCurrentSamples(double now_sec, const std::unordered_map<std::string, JointDiagState>& diag_map);
    FrameLayout computeFrameLayout();
    FrameData collectFrameData(double now_sec);
    void renderSceneFrame(const FrameLayout& layout);
    void renderSidebar(const FrameLayout& layout, const FrameData& frame);
    bool publishRcVirtualJoyCommand(const std::string& source, const std::string& note = "");
    void pushRcCommandLog(double now_sec, const std::string& source, bool ok, int active_locks, const std::string& note);

    ViewerConfig config_;

    GLFWwindow* window_ = nullptr;
    unsigned int shader_ = 0;

    OrbitCamera camera_;
    RobotScene scene_;
    SensorSubscriber sensor_subscriber_;

    bool sensor_ready_                = false;
    bool joy_ready_                   = false;
    bool omnilink_state_ready_        = false;
    bool wbc_ready_                   = false;
    bool robot_error_ready_           = false;
    bool rc_virtual_joy_ready_        = false;
    bool use_sensor_to_drive_robot_   = true;
    bool only_show_master_arm_groups_ = true;
    bool fix_base_like_mujoco_        = true;

    int side_panel_width_ = 560;
    int collapsed_sidebar_width_ = 34;
    bool sidebar_collapsed_ = false;

    std::vector<SensorJointSample> latest_sensor_samples_;
    std::vector<JoyButtonSample> latest_joy_buttons_;
    std::vector<JoyAxisSample> latest_joy_axes_;
    std::vector<JoyButtonSample> latest_state_buttons_;
    std::vector<JoyAxisSample> latest_state_axes_;
    std::vector<WbcGroupErrorSample> latest_wbc_group_errors_;
    int latest_wbc_joint_name_count_ = 0;
    int latest_wbc_state_pos_count_ = 0;
    std::vector<RobotErrorSample> latest_robot_errors_;
    std::unordered_map<std::string, int> rc_virtual_joy_command_;
    std::unordered_map<std::string, double> rc_virtual_joy_changed_at_s_;
    double last_rc_virtual_joy_send_s_ = -1.0;
    uint64_t rc_virtual_joy_send_count_ = 0;
    std::string rc_virtual_joy_last_error_;
    std::unordered_map<std::string, int> prev_joy_button_status_;
    std::deque<JoyButtonLogEntry> joy_button_logs_;
    size_t joy_button_log_limit_ = 120;
    float rc_virtual_joy_effect_timeout_s_ = 1.0f;
    std::deque<RcCommandLogEntry> rc_virtual_joy_logs_;
    size_t rc_virtual_joy_log_limit_ = 30;
    bool rc_inspect_running_ = false;
    bool rc_inspect_lock_phase_ = true;
    size_t rc_inspect_index_ = 0;
    double rc_inspect_step_begin_s_ = -1.0;
    float rc_inspect_step_duration_s_ = 0.8f;
    float rc_inspect_timeout_s_ = 0.8f;
    std::vector<std::string> rc_inspect_order_;
    std::unordered_map<std::string, RcInspectResult> rc_inspect_results_;
    std::string rc_inspect_last_message_;
    double rc_inspect_last_finish_s_ = -1.0;
    double omnilink_clean_streak_start_s_ = -1.0;
    double omnilink_best_clean_streak_s_ = 0.0;
    bool auto_lock_on_critical_fault_ = false;
    bool auto_lock_latched_ = false;
    std::string auto_lock_reason_;
    double auto_lock_time_s_ = -1.0;
    std::unordered_map<std::string, std::deque<float>> waveform_history_;
    std::string selected_wave_group_;
    std::string selected_wave_joint_;
    int selected_wave_metric_ = 0;
    int waveform_history_size_ = 600;
    float waveform_plot_height_ = 180.0f;
    std::unordered_map<std::string, double> baseline_position_rad_;
    bool baseline_ready_      = false;
    double baseline_capture_s = 0.0;
    float baseline_warn_deg_  = 15.0f;
    std::unordered_map<std::string, int> bad_frame_streak_;
    std::unordered_map<std::string, AlarmEntry> alarms_;
    bool show_only_active_alarms_ = true;
    int alarm_trigger_frames_      = 5;
    std::chrono::steady_clock::time_point app_start_time_;
    bool recording_         = false;
    std::ofstream record_file_;
    std::string record_file_path_;
    std::string record_output_dir_ = "logs";
    uint64_t recorded_rows_ = 0;
    std::string record_error_;
    uint64_t last_msg_count_seen_ = 0;
    std::deque<double> msg_arrival_times_sec_;
    double input_rate_hz_ = 0.0;

    double scroll_offset_ = 0.0;
    bool dragging_        = false;
    double last_x_        = 0.0;
    double last_y_        = 0.0;
    CameraDragMode drag_mode_ = CameraDragMode::None;
};

}  // namespace omnilink::teleop_viewer
