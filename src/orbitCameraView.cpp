#include "orbitCameraView.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

namespace omnilink::gui {
    glm::mat4 OrbitCameraView::getViewMatrix() const {
        float x = distance_ * cosf(pitch_) * cosf(yaw_);
        float y = distance_ * cosf(pitch_) * sinf(yaw_);
        float z = distance_ * sinf(pitch_);

        glm::vec3 eye = target_ + glm::vec3(x, y, z);
        return glm::lookAt(eye, target_, glm::vec3(0, 0, 1));
    }

    void MouseCtrlCamView::updateMouseCameraControl() {
        // 直接访问全局变量或通过成员变量访问
        ImGuiIO& io = ImGui::GetIO();

        if (!io.WantCaptureMouse) {
            GLFWwindow* window = glfwGetCurrentContext();  // 获取当前上下文窗口
            // 鼠标右键拖动旋转相机
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                if (!rotating_) {
                    rotating_ = true;
                    lastX_    = xpos;
                    lastY_    = ypos;
                } else {
                    double dx = xpos - lastX_;
                    double dy = ypos - lastY_;
                    lastX_    = xpos;
                    lastY_    = ypos;

                    camera.yaw_ += 0.005f * (float)dx;
                    camera.pitch_ += 0.005f * (float)dy;
                    if (camera.pitch_ > 1.5f)
                        camera.pitch_ = 1.5f;
                    if (camera.pitch_ < -1.5f)
                        camera.pitch_ = -1.5f;
                }
            } else {
                rotating_ = false;
            }
        }
    }
}  // namespace omnilink::gui