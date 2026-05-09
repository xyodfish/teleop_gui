#include "kinematic_viewer/kinematic_ros_bridge.h"

namespace kinematic_viewer {

KinematicRosBridge::KinematicRosBridge(bool enabled) : enabled_(enabled) {}

bool KinematicRosBridge::initialize(int argc, char** argv) {
    if (!enabled_) {
        return true;
    }
    if (!ros::isInitialized()) {
        ros::init(argc, argv, "robot_kinematic_viewer", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }
    node_handle_ = std::make_unique<ros::NodeHandle>("~");
    return true;
}

bool KinematicRosBridge::enabled() const {
    return enabled_;
}

void KinematicRosBridge::spinOnce() const {
    if (enabled_ && ros::ok()) {
        ros::spinOnce();
    }
}

double KinematicRosBridge::nowSec() const {
    if (!enabled_) {
        return 0.0;
    }
    return ros::Time::now().toSec();
}

void KinematicRosBridge::shutdown() const {
    if (!enabled_) {
        return;
    }
    if (ros::isStarted()) {
        ros::shutdown();
    }
}

}  // namespace kinematic_viewer
