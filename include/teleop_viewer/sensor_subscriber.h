#pragma once

#include <embosa.hpp>
#include <galbot/singorix_proto/singorix_sensor.pb.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "teleop_viewer/types.h"

namespace omnilink::teleop_viewer {

class SensorSubscriber {
   public:
    ~SensorSubscriber();

    bool start(const std::string& topic, const std::string& node_name);
    void stop();

    std::vector<SensorJointSample> latestSamples() const;

    uint64_t messageCount() const;
    double messageAgeSec() const;
    bool hasRecentData(double timeout_sec) const;

   private:
    static long long nowSteadyNs();
    void onMessage(const std::shared_ptr<galbot::singorix_proto::SingoriXSensor>& msg);

    std::string topic_;
    std::unique_ptr<galbot::embosa::Node> node_;
    std::shared_ptr<galbot::embosa::SerializationReader<galbot::singorix_proto::SingoriXSensor>> reader_;

    mutable std::mutex data_mtx_;
    std::vector<SensorJointSample> latest_samples_;

    std::atomic<uint64_t> message_count_{0};
    std::atomic<long long> last_msg_ns_{0};

    bool embosa_inited_ = false;
    bool running_       = false;
};

}  // namespace omnilink::teleop_viewer
