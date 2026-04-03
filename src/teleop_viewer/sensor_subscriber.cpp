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

void SensorSubscriber::stop() {
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

}  // namespace omnilink::teleop_viewer
