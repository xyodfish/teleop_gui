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
        float distance   = 3.0f;
        float yaw        = 0.0f;
        float pitch      = 0.0f;
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
            float position  = 0.0f;
            float min_angle = -3.14f;
            float max_angle = 3.14f;
            bool revolute   = false;
        };
        struct JointAxisInfo {
            std::string name;
            glm::vec3 world_origin = glm::vec3(0.0f);
            glm::vec3 world_axis   = glm::vec3(0.0f, 0.0f, 1.0f);
            bool revolute          = false;
        };
        struct LinkTfInfo {
            std::string name;
            std::string parent_name;
            glm::vec3 world_position = glm::vec3(0.0f);
            glm::vec3 world_rpy      = glm::vec3(0.0f);
        };
        struct LinkCollisionProxy {
            std::string link_name;
            std::string visual_name;
            glm::vec3 world_center = glm::vec3(0.0f);
            float radius_m         = 0.0f;
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
        std::vector<JointAxisInfo> getJointAxisInfos(bool revolute_only = true) const;
        std::vector<LinkTfInfo> getLinkTfInfos() const;
        std::vector<LinkCollisionProxy> getLinkCollisionProxies() const;
        bool getLinkWorldTransform(const std::string& link_name, glm::mat4* out_world_transform) const;
        bool getLinkParentName(const std::string& link_name, std::string* out_parent_name) const;

        void setFixedBaseMode(bool enabled);
        bool fixedBaseMode() const;
        void setVirtualBasePose2D(float x_m, float y_m, float yaw_rad);
        bool getVirtualBasePose2D(float* x_m, float* y_m, float* yaw_rad) const;

       private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace omnilink::teleop_viewer
