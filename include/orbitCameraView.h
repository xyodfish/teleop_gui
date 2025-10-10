#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

namespace omnilink::gui {
    class OrbitCameraView {
       public:
        float distance_   = 3.0f;                // 相机距离
        float yaw_        = 0.0f;                // 水平角
        float pitch_      = 0.0f;                // 垂直角
        glm::vec3 target_ = glm::vec3(0, 0, 0);  // 相机观察的目标点
        glm::mat4 getViewMatrix() const;
    };

    class MouseCtrlCamView {
       public:
        OrbitCameraView camera;
        double lastX_ = 0.0, lastY_ = 0.0;
        bool rotating_       = false;
        double scrollOffset_ = 0.0;

        void updateMouseCameraControl();
    };
}  // namespace omnilink::gui