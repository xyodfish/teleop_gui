#pragma once

#include <string>

namespace omnilink::teleop_viewer {

struct SensorJointSample {
    std::string group;
    std::string name;
    double position = 0.0;
    double velocity = 0.0;
    double effort   = 0.0;
    double current  = 0.0;

    bool has_position = false;
    bool has_velocity = false;
    bool has_effort   = false;
    bool has_current  = false;
};

inline bool IsMasterArmGroup(const std::string& group_name) {
    return group_name == "left_arm" || group_name == "right_arm";
}

inline bool IsBaseMotionJointName(const std::string& joint_name) {
    return joint_name.find("chassis") != std::string::npos || joint_name.find("world") != std::string::npos ||
           joint_name.find("virtual") != std::string::npos || joint_name.find("base") != std::string::npos;
}

}  // namespace omnilink::teleop_viewer
