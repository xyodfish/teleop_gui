#include "kinematic_viewer/kinematic_marker_utils.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace kinematic_viewer {

glm::mat4 markerWorldMatrix(const glm::vec3& pos, const glm::vec3& rpyDeg) {
    glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
    m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[0]), glm::vec3(1.0f, 0.0f, 0.0f));
    m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[1]), glm::vec3(0.0f, 1.0f, 0.0f));
    m           = m * glm::rotate(glm::mat4(1.0f), glm::radians(rpyDeg[2]), glm::vec3(0.0f, 0.0f, 1.0f));
    return m;
}

float wrapDeltaDeg(float deltaDeg) {
    float wrapped = std::fmod(deltaDeg + 180.0f, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

void appendMarkerAxes(std::vector<KinematicLineVertex>* out, const glm::vec3& pos, const glm::vec3& rpyDeg, float axisLen, bool selected) {
    if (out == nullptr) {
        return;
    }
    const glm::mat4 rot    = markerWorldMatrix(glm::vec3(0.0f), rpyDeg);
    const glm::vec3 xAxis  = glm::normalize(glm::vec3(rot * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
    const glm::vec3 yAxis  = glm::normalize(glm::vec3(rot * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
    const glm::vec3 zAxis  = glm::normalize(glm::vec3(rot * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
    const float scale      = selected ? 1.0f : 0.75f;
    const glm::vec3 colorX = selected ? glm::vec3(1.0f, 0.25f, 0.25f) : glm::vec3(0.85f, 0.45f, 0.45f);
    const glm::vec3 colorY = selected ? glm::vec3(0.25f, 1.0f, 0.25f) : glm::vec3(0.45f, 0.85f, 0.45f);
    const glm::vec3 colorZ = selected ? glm::vec3(0.25f, 0.55f, 1.0f) : glm::vec3(0.45f, 0.65f, 0.85f);

    out->push_back({pos, colorX});
    out->push_back({pos + xAxis * axisLen * scale, colorX});
    out->push_back({pos, colorY});
    out->push_back({pos + yAxis * axisLen * scale, colorY});
    out->push_back({pos, colorZ});
    out->push_back({pos + zAxis * axisLen * scale, colorZ});
}

void appendCircle(std::vector<KinematicLineVertex>* out, const glm::vec3& center, const glm::vec3& normal, float radius, const glm::vec3& color,
                  int segments) {
    if (out == nullptr || segments < 8) {
        return;
    }
    glm::vec3 normalUnit = glm::normalize(normal);
    glm::vec3 helper     = std::fabs(normalUnit.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 u          = glm::normalize(glm::cross(normalUnit, helper));
    glm::vec3 v          = glm::normalize(glm::cross(normalUnit, u));
    for (int i = 0; i < segments; ++i) {
        float t0     = (2.0f * 3.1415926f * static_cast<float>(i)) / static_cast<float>(segments);
        float t1     = (2.0f * 3.1415926f * static_cast<float>(i + 1)) / static_cast<float>(segments);
        glm::vec3 p0 = center + radius * (std::cos(t0) * u + std::sin(t0) * v);
        glm::vec3 p1 = center + radius * (std::cos(t1) * u + std::sin(t1) * v);
        out->push_back({p0, color});
        out->push_back({p1, color});
    }
}

glm::vec2 worldToScreen(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, int viewportW, int viewportH, bool* ok) {
    glm::vec4 clip = proj * view * glm::vec4(p, 1.0f);
    if (std::fabs(clip.w) < 1e-6f) {
        if (ok) {
            *ok = false;
        }
        return glm::vec2(-1.0f);
    }
    glm::vec3 ndc = glm::vec3(clip) / clip.w;
    if (ok) {
        *ok = true;
    }
    return glm::vec2((ndc.x * 0.5f + 0.5f) * static_cast<float>(viewportW), (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(viewportH));
}

float distancePointToSegment2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    float ab2    = glm::dot(ab, ab);
    if (ab2 < 1e-6f) {
        return glm::length(p - a);
    }
    float t        = glm::clamp(glm::dot(p - a, ab) / ab2, 0.0f, 1.0f);
    glm::vec2 proj = a + t * ab;
    return glm::length(p - proj);
}

}  // namespace kinematic_viewer
