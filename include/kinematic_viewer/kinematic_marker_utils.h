#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <vector>

namespace kinematic_viewer {

struct KinematicLineVertex {
    glm::vec3 p;
    glm::vec3 c;
};

glm::mat4 markerWorldMatrix(const glm::vec3& pos, const glm::vec3& rpyDeg);
float wrapDeltaDeg(float deltaDeg);
void appendMarkerAxes(std::vector<KinematicLineVertex>* out, const glm::vec3& pos, const glm::vec3& rpyDeg, float axisLen, bool selected);
void appendCircle(std::vector<KinematicLineVertex>* out, const glm::vec3& center, const glm::vec3& normal, float radius, const glm::vec3& color,
                  int segments);
glm::vec2 worldToScreen(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, int viewportW, int viewportH, bool* ok);
float distancePointToSegment2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b);

}  // namespace kinematic_viewer
