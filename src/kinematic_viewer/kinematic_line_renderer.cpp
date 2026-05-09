#include "kinematic_viewer/kinematic_line_renderer.h"

#include <cstddef>

namespace kinematic_viewer {

void KinematicLineRenderer::init() {
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
}

void KinematicLineRenderer::draw(GLuint shader, const std::vector<KinematicLineVertex>& vertices, const glm::mat4& view, const glm::mat4& proj,
                                 float lineWidth) {
    if (vertices.empty()) {
        return;
    }
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, &proj[0][0]);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(KinematicLineVertex)), vertices.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(KinematicLineVertex), (void*)offsetof(KinematicLineVertex, p));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(KinematicLineVertex), (void*)offsetof(KinematicLineVertex, c));

    glLineWidth(lineWidth);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
    glBindVertexArray(0);
}

KinematicLineRenderer::~KinematicLineRenderer() {
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
    }
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
    }
}

}  // namespace kinematic_viewer
