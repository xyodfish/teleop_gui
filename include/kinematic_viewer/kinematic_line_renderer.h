#pragma once

#include "kinematic_viewer/kinematic_marker_utils.h"

#include <glad/glad.h>
#include <glm/mat4x4.hpp>

#include <vector>

namespace kinematic_viewer {

class KinematicLineRenderer {
   public:
    void init();
    void draw(GLuint shader, const std::vector<KinematicLineVertex>& vertices, const glm::mat4& view, const glm::mat4& proj, float lineWidth);
    ~KinematicLineRenderer();

   private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
};

}  // namespace kinematic_viewer
