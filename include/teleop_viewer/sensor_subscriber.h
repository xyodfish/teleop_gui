#pragma once

#include <embosa.hpp>
#include <galbot/singorix_proto/singorix_sensor.pb.h>
#include <galbot/sensor_proto/joy.pb.h>

#include <atomic>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "teleop_viewer/msg_publisher.h"
#include "teleop_viewer/types.h"

namespace omnilink::teleop_viewer {

class SensorSubscriber {
   public:
    ~SensorSubscriber();

    bool start(const std::string& topic, const std::string& node_name);
    bool startJoy(const std::string& topic);
    bool startOmnilinkStates(const std::string& topic);
    bool startRcVirtualJoyPublisher(const std::string& topic);
    bool publishRcVirtualJoyButtons(const std::vector<std::pair<std::string, int>>& buttons);
    void stop();

    std::vector<SensorJointSample> latestSamples() const;
    std::vector<JoyButtonSample> latestJoyButtons() const;
    std::vector<JoyAxisSample> latestJoyAxes() const;
    std::vector<JoyButtonSample> latestStateButtons() const;
    std::vector<JoyAxisSample> latestStateAxes() const;

    uint64_t messageCount() const;
    double messageAgeSec() const;
    bool hasRecentData(double timeout_sec) const;
    uint64_t joyMessageCount() const;
    double joyMessageAgeSec() const;
    bool joyHasRecentData(double timeout_sec) const;
    uint64_t stateMessageCount() const;
    double stateMessageAgeSec() const;
    bool stateHasRecentData(double timeout_sec) const;

   private:
    static long long nowSteadyNs();
    void onMessage(const std::shared_ptr<galbot::singorix_proto::SingoriXSensor>& msg);
    void onJoyMessage(const std::shared_ptr<galbot::sensor_proto::Joy>& msg);
    void onStateMessage(const std::shared_ptr<galbot::sensor_proto::Joy>& msg);

    std::string topic_;
    std::string joy_topic_;
    std::string state_topic_;
    std::string rc_virtual_joy_topic_;
    std::unique_ptr<galbot::embosa::Node> node_;
    std::shared_ptr<galbot::embosa::SerializationReader<galbot::singorix_proto::SingoriXSensor>> reader_;
    std::shared_ptr<galbot::embosa::SerializationReader<galbot::sensor_proto::Joy>> joy_reader_;
    std::shared_ptr<galbot::embosa::SerializationReader<galbot::sensor_proto::Joy>> state_reader_;
    MsgPublisher msg_publisher_;

    mutable std::mutex data_mtx_;
    std::vector<SensorJointSample> latest_samples_;
    std::vector<JoyButtonSample> latest_joy_buttons_;
    std::vector<JoyAxisSample> latest_joy_axes_;
    std::vector<JoyButtonSample> latest_state_buttons_;
    std::vector<JoyAxisSample> latest_state_axes_;

    std::atomic<uint64_t> message_count_{0};
    std::atomic<long long> last_msg_ns_{0};
    std::atomic<uint64_t> joy_message_count_{0};
    std::atomic<long long> joy_last_msg_ns_{0};
    std::atomic<uint64_t> state_message_count_{0};
    std::atomic<long long> state_last_msg_ns_{0};

    bool embosa_inited_ = false;
    bool running_       = false;
};

}  // namespace omnilink::teleop_viewer
