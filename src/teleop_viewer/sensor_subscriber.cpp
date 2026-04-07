#include "teleop_viewer/sensor_subscriber.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

namespace omnilink::teleop_viewer {

    SensorSubscriber::~SensorSubscriber() { stop(); }

    bool SensorSubscriber::start(const std::string& topic, const std::string& node_name) {
        if (running_) {
            return true;
        }

        topic_ = topic;

        // Some embosa builds do not export EmbosaInit(), but do export EmbosaInitInternal().
        embosa_inited_ = galbot::embosa::EmbosaInitInternal();
        if (!embosa_inited_) {
            std::cerr << "EmbosaInit failed, sensor monitor disabled." << std::endl;
            return false;
        }

        node_ = galbot::embosa::CreateNode(node_name);
        if (!node_) {
            std::cerr << "CreateNode failed, sensor monitor disabled." << std::endl;
            stop();
            return false;
        }

        reader_ = node_->CreateReader<galbot::singorix_proto::SingoriXSensor>(
            topic_, [this](const std::shared_ptr<galbot::singorix_proto::SingoriXSensor>& msg, const void*) { onMessage(msg); });

        if (!reader_) {
            std::cerr << "CreateReader for topic [" << topic_ << "] failed." << std::endl;
            stop();
            return false;
        }

        running_ = true;
        return true;
    }

    bool SensorSubscriber::startJoy(const std::string& topic) {
        if (!running_ || !node_) {
            std::cerr << "startJoy failed: sensor subscriber is not initialized." << std::endl;
            return false;
        }

        joy_topic_  = topic;
        joy_reader_ = node_->CreateReader<galbot::sensor_proto::Joy>(
            joy_topic_, [this](const std::shared_ptr<galbot::sensor_proto::Joy>& msg, const void*) { onJoyMessage(msg); });

        if (!joy_reader_) {
            std::cerr << "CreateReader for joy topic [" << joy_topic_ << "] failed." << std::endl;
            return false;
        }
        return true;
    }

    bool SensorSubscriber::startOmnilinkStates(const std::string& topic) {
        if (!running_ || !node_) {
            std::cerr << "startOmnilinkStates failed: sensor subscriber is not initialized." << std::endl;
            return false;
        }

        state_topic_  = topic;
        state_reader_ = node_->CreateReader<galbot::sensor_proto::Joy>(
            state_topic_, [this](const std::shared_ptr<galbot::sensor_proto::Joy>& msg, const void*) { onStateMessage(msg); });

        if (!state_reader_) {
            std::cerr << "CreateReader for omnilink state topic [" << state_topic_ << "] failed." << std::endl;
            return false;
        }
        return true;
    }

    bool SensorSubscriber::startWbcInfo(const std::string& topic) {
        if (!running_ || !node_) {
            std::cerr << "startWbcInfo failed: sensor subscriber is not initialized." << std::endl;
            return false;
        }

        wbc_topic_  = topic;
        wbc_reader_ = node_->CreateReader<galbot::singorix_proto::WBCInfo>(
            wbc_topic_, [this](const std::shared_ptr<galbot::singorix_proto::WBCInfo>& msg, const void*) { onWbcInfoMessage(msg); });

        if (!wbc_reader_) {
            std::cerr << "CreateReader for wbc info topic [" << wbc_topic_ << "] failed." << std::endl;
            return false;
        }
        return true;
    }

    bool SensorSubscriber::startRobotErrors(const std::string& topic) {
        if (!running_ || !node_) {
            std::cerr << "startRobotErrors failed: sensor subscriber is not initialized." << std::endl;
            return false;
        }

        error_topic_  = topic;
        error_reader_ = node_->CreateReader<galbot::singorix_proto::SingoriXError>(
            error_topic_,
            [this](const std::shared_ptr<galbot::singorix_proto::SingoriXError>& msg, const void*) { onErrorMessage(msg); });

        if (!error_reader_) {
            std::cerr << "CreateReader for robot error topic [" << error_topic_ << "] failed." << std::endl;
            return false;
        }
        return true;
    }

    bool SensorSubscriber::startRcVirtualJoyPublisher(const std::string& topic) {
        if (!running_ || !node_) {
            std::cerr << "startRcVirtualJoyPublisher failed: sensor subscriber is not initialized." << std::endl;
            return false;
        }

        rc_virtual_joy_topic_  = topic;
        if (!msg_publisher_.initRcVirtualJoyWriter(node_.get(), rc_virtual_joy_topic_)) {
            std::cerr << "CreateWriter for rc virtual joy topic [" << rc_virtual_joy_topic_ << "] failed." << std::endl;
            return false;
        }
        return true;
    }

    bool SensorSubscriber::publishRcVirtualJoyButtons(const std::vector<std::pair<std::string, int>>& buttons) {
        if (!running_ || !node_) {
            return false;
        }
        return msg_publisher_.publishRcVirtualJoyButtons(buttons);
    }

    void SensorSubscriber::stop() {
        msg_publisher_.reset();
        error_reader_.reset();
        wbc_reader_.reset();
        state_reader_.reset();
        joy_reader_.reset();
        reader_.reset();
        node_.reset();

        if (embosa_inited_) {
            galbot::embosa::Clear();
            embosa_inited_ = false;
        }

        running_ = false;
    }

    std::vector<SensorJointSample> SensorSubscriber::latestSamples() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_samples_;
    }

    std::vector<JoyButtonSample> SensorSubscriber::latestJoyButtons() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_joy_buttons_;
    }

    std::vector<JoyAxisSample> SensorSubscriber::latestJoyAxes() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_joy_axes_;
    }

    std::vector<JoyButtonSample> SensorSubscriber::latestStateButtons() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_state_buttons_;
    }

    std::vector<JoyAxisSample> SensorSubscriber::latestStateAxes() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_state_axes_;
    }

    std::vector<WbcGroupErrorSample> SensorSubscriber::latestWbcGroupErrors() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_wbc_group_errors_;
    }

    std::vector<RobotErrorSample> SensorSubscriber::latestRobotErrors() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_robot_errors_;
    }

    int SensorSubscriber::latestWbcJointNameCount() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_wbc_joint_name_count_;
    }

    int SensorSubscriber::latestWbcStatePosCount() const {
        std::lock_guard<std::mutex> lock(data_mtx_);
        return latest_wbc_state_pos_count_;
    }

    uint64_t SensorSubscriber::messageCount() const { return message_count_.load(std::memory_order_acquire); }

    double SensorSubscriber::messageAgeSec() const {
        long long ts_ns = last_msg_ns_.load(std::memory_order_acquire);
        if (ts_ns <= 0) {
            return -1.0;
        }

        long long now_ns = nowSteadyNs();
        return static_cast<double>(now_ns - ts_ns) / 1e9;
    }

    bool SensorSubscriber::hasRecentData(double timeout_sec) const {
        if (messageCount() == 0) {
            return false;
        }
        double age = messageAgeSec();
        return age >= 0.0 && age <= timeout_sec;
    }

    uint64_t SensorSubscriber::joyMessageCount() const { return joy_message_count_.load(std::memory_order_acquire); }

    double SensorSubscriber::joyMessageAgeSec() const {
        long long ts_ns = joy_last_msg_ns_.load(std::memory_order_acquire);
        if (ts_ns <= 0) {
            return -1.0;
        }
        long long now_ns = nowSteadyNs();
        return static_cast<double>(now_ns - ts_ns) / 1e9;
    }

    bool SensorSubscriber::joyHasRecentData(double timeout_sec) const {
        if (joyMessageCount() == 0) {
            return false;
        }
        double age = joyMessageAgeSec();
        return age >= 0.0 && age <= timeout_sec;
    }

    uint64_t SensorSubscriber::stateMessageCount() const { return state_message_count_.load(std::memory_order_acquire); }

    double SensorSubscriber::stateMessageAgeSec() const {
        long long ts_ns = state_last_msg_ns_.load(std::memory_order_acquire);
        if (ts_ns <= 0) {
            return -1.0;
        }
        long long now_ns = nowSteadyNs();
        return static_cast<double>(now_ns - ts_ns) / 1e9;
    }

    bool SensorSubscriber::stateHasRecentData(double timeout_sec) const {
        if (stateMessageCount() == 0) {
            return false;
        }
        double age = stateMessageAgeSec();
        return age >= 0.0 && age <= timeout_sec;
    }

    uint64_t SensorSubscriber::wbcMessageCount() const { return wbc_message_count_.load(std::memory_order_acquire); }

    double SensorSubscriber::wbcMessageAgeSec() const {
        long long ts_ns = wbc_last_msg_ns_.load(std::memory_order_acquire);
        if (ts_ns <= 0) {
            return -1.0;
        }
        long long now_ns = nowSteadyNs();
        return static_cast<double>(now_ns - ts_ns) / 1e9;
    }

    bool SensorSubscriber::wbcHasRecentData(double timeout_sec) const {
        if (wbcMessageCount() == 0) {
            return false;
        }
        double age = wbcMessageAgeSec();
        return age >= 0.0 && age <= timeout_sec;
    }

    uint64_t SensorSubscriber::errorMessageCount() const { return error_message_count_.load(std::memory_order_acquire); }

    double SensorSubscriber::errorMessageAgeSec() const {
        long long ts_ns = error_last_msg_ns_.load(std::memory_order_acquire);
        if (ts_ns <= 0) {
            return -1.0;
        }
        long long now_ns = nowSteadyNs();
        return static_cast<double>(now_ns - ts_ns) / 1e9;
    }

    bool SensorSubscriber::errorHasRecentData(double timeout_sec) const {
        if (errorMessageCount() == 0) {
            return false;
        }
        double age = errorMessageAgeSec();
        return age >= 0.0 && age <= timeout_sec;
    }

    long long SensorSubscriber::nowSteadyNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void SensorSubscriber::onMessage(const std::shared_ptr<galbot::singorix_proto::SingoriXSensor>& msg) {
        if (!msg) {
            return;
        }

        std::vector<SensorJointSample> parsed_samples;
        parsed_samples.reserve(64);

        for (const auto& group_item : msg->joint_sensor_map()) {
            const std::string& group_name = group_item.first;
            const auto& joint_sensor      = group_item.second;
            int n                         = joint_sensor.name_size();

            for (int i = 0; i < n; ++i) {
                SensorJointSample sample;
                sample.group = group_name;
                sample.name  = joint_sensor.name(i);
                if (sample.name.empty()) {
                    continue;
                }

                if (i < joint_sensor.position_size()) {
                    sample.position     = joint_sensor.position(i);
                    sample.has_position = true;
                }
                if (i < joint_sensor.velocity_size()) {
                    sample.velocity     = joint_sensor.velocity(i);
                    sample.has_velocity = true;
                }
                if (i < joint_sensor.effort_size()) {
                    sample.effort     = joint_sensor.effort(i);
                    sample.has_effort = true;
                }
                if (i < joint_sensor.current_size()) {
                    sample.current     = joint_sensor.current(i);
                    sample.has_current = true;
                }

                parsed_samples.push_back(std::move(sample));
            }
        }

        {
            std::lock_guard<std::mutex> lock(data_mtx_);
            latest_samples_.swap(parsed_samples);
        }

        message_count_.fetch_add(1, std::memory_order_acq_rel);
        last_msg_ns_.store(nowSteadyNs(), std::memory_order_release);
    }

    void SensorSubscriber::onJoyMessage(const std::shared_ptr<galbot::sensor_proto::Joy>& msg) {
        if (!msg) {
            return;
        }

        std::vector<JoyButtonSample> buttons;
        std::vector<JoyAxisSample> axes;

        buttons.reserve(msg->button_list_size());
        axes.reserve(msg->axes_list_size());

        for (const auto& button : msg->button_list()) {
            JoyButtonSample item;
            item.name   = button.header().frame_id();
            item.status = button.status();
            buttons.push_back(std::move(item));
        }

        for (const auto& axis : msg->axes_list()) {
            JoyAxisSample item;
            item.name      = axis.header().frame_id();
            item.value     = axis.value();
            item.has_value = std::isfinite(item.value);
            axes.push_back(std::move(item));
        }

        {
            std::lock_guard<std::mutex> lock(data_mtx_);
            latest_joy_buttons_.swap(buttons);
            latest_joy_axes_.swap(axes);
        }

        joy_message_count_.fetch_add(1, std::memory_order_acq_rel);
        joy_last_msg_ns_.store(nowSteadyNs(), std::memory_order_release);
    }

    void SensorSubscriber::onStateMessage(const std::shared_ptr<galbot::sensor_proto::Joy>& msg) {
        if (!msg) {
            return;
        }

        std::vector<JoyButtonSample> buttons;
        std::vector<JoyAxisSample> axes;

        buttons.reserve(msg->button_list_size());
        axes.reserve(msg->axes_list_size());

        for (const auto& button : msg->button_list()) {
            JoyButtonSample item;
            item.name   = button.header().frame_id();
            item.status = button.status();
            buttons.push_back(std::move(item));
        }

        for (const auto& axis : msg->axes_list()) {
            JoyAxisSample item;
            item.name      = axis.header().frame_id();
            item.value     = axis.value();
            item.has_value = std::isfinite(item.value);
            axes.push_back(std::move(item));
        }

        {
            std::lock_guard<std::mutex> lock(data_mtx_);
            latest_state_buttons_.swap(buttons);
            latest_state_axes_.swap(axes);
        }

        state_message_count_.fetch_add(1, std::memory_order_acq_rel);
        state_last_msg_ns_.store(nowSteadyNs(), std::memory_order_release);
    }

    void SensorSubscriber::onWbcInfoMessage(const std::shared_ptr<galbot::singorix_proto::WBCInfo>& msg) {
        if (!msg) {
            return;
        }

        std::vector<WbcGroupErrorSample> groups;
        groups.reserve(static_cast<size_t>(msg->group_info_map_size()));
        for (const auto& [group_name, group_info] : msg->group_info_map()) {
            WbcGroupErrorSample item;
            item.group = group_name;
            item.joint_count = group_info.joint_names_size();
            item.pos_norm = group_info.error_final_pos_norm();
            item.vel_norm = group_info.error_final_vel_norm();
            item.eff_norm = group_info.error_final_eff_norm();
            groups.push_back(std::move(item));
        }

        int joint_name_count = 0;
        int state_pos_count = 0;
        if (msg->has_joint_info()) {
            joint_name_count = msg->joint_info().names_size();
            state_pos_count = msg->joint_info().state_pos_size();
        }

        {
            std::lock_guard<std::mutex> lock(data_mtx_);
            latest_wbc_group_errors_.swap(groups);
            latest_wbc_joint_name_count_ = joint_name_count;
            latest_wbc_state_pos_count_ = state_pos_count;
        }

        wbc_message_count_.fetch_add(1, std::memory_order_acq_rel);
        wbc_last_msg_ns_.store(nowSteadyNs(), std::memory_order_release);
    }

    void SensorSubscriber::onErrorMessage(const std::shared_ptr<galbot::singorix_proto::SingoriXError>& msg) {
        if (!msg) {
            return;
        }

        std::vector<RobotErrorSample> errors;
        errors.reserve(msg->error_map().size());
        for (const auto& [component, detail] : msg->error_map()) {
            RobotErrorSample item;
            item.component = component;
            item.code = detail.error_code();
            item.description = detail.description();
            errors.push_back(std::move(item));
        }

        {
            std::lock_guard<std::mutex> lock(data_mtx_);
            latest_robot_errors_.swap(errors);
        }

        error_message_count_.fetch_add(1, std::memory_order_acq_rel);
        error_last_msg_ns_.store(nowSteadyNs(), std::memory_order_release);
    }

}  // namespace omnilink::teleop_viewer
