#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>

namespace kinematic_viewer {

class KinematicRosBridge {
   public:
    explicit KinematicRosBridge(bool enabled);

    bool initialize(int argc, char** argv);
    bool enabled() const;
    void spinOnce() const;
    double nowSec() const;
    void shutdown() const;

    template <typename T>
    bool getParam(const std::string& key, T* out) const {
        if (!enabled_ || node_handle_ == nullptr || out == nullptr) {
            return false;
        }
        return node_handle_->getParam(key, *out);
    }

    template <typename T>
    void param(const std::string& key, T* value, const T& default_value) const {
        if (value == nullptr) {
            return;
        }
        if (!enabled_ || node_handle_ == nullptr) {
            *value = default_value;
            return;
        }
        node_handle_->param<T>(key, *value, default_value);
    }

    template <typename Callback>
    void subscribeExternalTarget(const std::string& topic, uint32_t queue_size, Callback&& callback) {
        if (!enabled_ || node_handle_ == nullptr) {
            return;
        }
        external_target_sub_ = node_handle_->subscribe<geometry_msgs::PoseStamped>(topic, queue_size, std::forward<Callback>(callback));
    }

   private:
    bool enabled_ = true;
    std::unique_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber external_target_sub_;
};

}  // namespace kinematic_viewer
