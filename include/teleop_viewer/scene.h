#pragma once

#include <glad/glad.h>

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

#include "teleop_viewer/types.h"

namespace omnilink::teleop_viewer {

class OrbitCamera {
   public:
    float distance = 3.0f;
    float yaw      = 0.0f;
    float pitch    = 0.0f;
    glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);

    float rotate_speed = 0.005f;
    float zoom_scale   = 0.1f;
    float dolly_scale  = 0.02f;
    float pan_scale    = 0.0015f;
    float min_distance = 0.2f;
    float max_distance = 20.0f;

    glm::vec3 eye() const;
    glm::mat4 viewMatrix() const;

    void rotate(float dx, float dy);
    void zoom(float delta);
    void dolly(float dy);
    void pan(float dx, float dy);
};

class RobotScene {
   public:
    struct JointInfo {
        std::string name;
        float position = 0.0f;
        float min_angle = -3.14f;
        float max_angle = 3.14f;
        bool revolute = false;
    };

    RobotScene();
    ~RobotScene();

    bool loadURDF(const std::string& urdf_path);

    void updateTransforms();
    void draw(GLuint shader);

    size_t applyJointSamples(const std::vector<SensorJointSample>& samples, bool only_master_arm);
    bool setJointPositionByName(const std::string& joint_name, float new_position);

    bool getJointInfo(const std::string& joint_name, JointInfo* out) const;
    std::vector<JointInfo> getJointInfos() const;

    void setFixedBaseMode(bool enabled);
    bool fixedBaseMode() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace omnilink::teleop_viewer
