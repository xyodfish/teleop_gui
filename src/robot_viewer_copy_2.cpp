// robot_viewer_fixed.cpp
#define STB_IMAGE_IMPLEMENTATION
#include <glad/glad.h>

#include "stb_image.h"

#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <urdf_parser/urdf_parser.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <sys/stat.h>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ======================== shader sources (unchanged) ========================
const char* vertex_shader_src = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoords = aTexCoords;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

const char* fragment_shader_src = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 diffuseColor;
uniform bool hasTexture;
uniform sampler2D texture_diffuse1;

out vec4 color;

void main()
{
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * diffuseColor;

    vec3 ambient = 0.5 * diffuseColor;

    vec3 result = ambient + diffuse;
    if (hasTexture) {
        result *= texture(texture_diffuse1, TexCoords).rgb;
    }
    color = vec4(result, 1.0);
}
)";

// ======================== mesh/model classes (unchanged except small fixes) ========================
struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};

struct Texture {
    unsigned int id;
    std::string type;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    glm::vec3 diffuseColor = glm::vec3(0.8f, 0.8f, 0.8f);
    unsigned int VAO = 0, VBO = 0, EBO = 0;

    void setup() {
        if (VAO) {  // 如果之前已经 setup，先删除
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
            glDeleteBuffers(1, &EBO);
            VAO = VBO = EBO = 0;
        }

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

        glBindVertexArray(0);
    }

    void draw(GLuint shader) {
        glBindVertexArray(VAO);
        if (!textures.empty()) {
            glUniform1i(glGetUniformLocation(shader, "hasTexture"), true);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textures[0].id);
        } else {
            glUniform1i(glGetUniformLocation(shader, "hasTexture"), false);
            glUniform3f(glGetUniformLocation(shader, "diffuseColor"), diffuseColor.r, diffuseColor.g, diffuseColor.b);
        }
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};

class Model {
   public:
    std::vector<Mesh> meshes;
    std::string directory;

    void loadAssimp(const std::string& path) {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
            return;
        }

        // 如果 path 包含目录，则记录 directory（供纹理等查找）
        size_t lastSlash = path.find_last_of('/');
        if (lastSlash != std::string::npos)
            directory = path.substr(0, lastSlash);

        processNode(scene->mRootNode, scene);
    }

   private:
    void processNode(aiNode* node, const aiScene* scene) {
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(mesh, scene));
        }

        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            processNode(node->mChildren[i], scene);
        }
    }

    Mesh processMesh(aiMesh* mesh, const aiScene* scene) {
        Mesh m;
        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            Vertex vertex;
            vertex.Position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
            if (mesh->HasNormals()) {
                vertex.Normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
            } else {
                vertex.Normal = glm::vec3(0.0f, 0.0f, 1.0f);
            }
            if (mesh->mTextureCoords[0]) {
                vertex.TexCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            } else {
                vertex.TexCoords = glm::vec2(0.0f, 0.0f);
            }
            m.vertices.push_back(vertex);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; j++)
                m.indices.push_back(face.mIndices[j]);
        }

        if (mesh->mMaterialIndex >= 0) {
            aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            aiColor3D color(1.0f, 1.0f, 1.0f);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
            m.diffuseColor = glm::vec3(color.r, color.g, color.b);

            if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
                aiString str;
                material->GetTexture(aiTextureType_DIFFUSE, 0, &str);
                std::string texPath = directory + "/" + str.C_Str();
                Texture tex;
                tex.id   = loadTexture(texPath);
                tex.type = "texture_diffuse";
                m.textures.push_back(tex);
            }
        }

        m.setup();
        return m;
    }

    unsigned int loadTexture(const std::string& path) {
        unsigned int textureID;
        glGenTextures(1, &textureID);
        int width, height, nrComponents;
        unsigned char* data = stbi_load(path.c_str(), &width, &height, &nrComponents, 0);
        if (data) {
            GLenum format = nrComponents == 1 ? GL_RED : nrComponents == 3 ? GL_RGB : GL_RGBA;
            glBindTexture(GL_TEXTURE_2D, textureID);
            glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            stbi_image_free(data);
        } else {
            std::cerr << "Texture failed to load at path: " << path << std::endl;
            stbi_image_free(data);
        }
        return textureID;
    }
};

// ======================== URDF state & Robot ========================

// 保存每个 visual 的本地偏移与所属 link
struct LinkVisual {
    std::string mesh_file;
    glm::vec3 origin_xyz = glm::vec3(0);
    glm::vec3 origin_rpy = glm::vec3(0);
    glm::vec3 scale      = glm::vec3(1);

    // <<< MOD: 父 link 名称 & visual 相对于 link 的本地变换（不包含 link 全局变换）
    std::string parent_link_name;
    glm::mat4 local_transform = glm::mat4(1.0f);

    Model model;
    bool loaded = false;
};

struct JointState {
    std::string name;
    float position  = 0.0f;  // radians
    float min_angle = -3.14f;
    float max_angle = 3.14f;
};

// 相机、鼠标控制（保持原样）
// ...（直接拷贝你原有 Camera 类）...
class Camera {
   public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;

    float yaw;
    float pitch;

    float movementSpeed;
    float mouseSensitivity;
    float zoom;

    Camera(glm::vec3 pos = glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = -90.0f, float pitch = 0.0f)
        : front(glm::vec3(0.0f, 0.0f, -1.0f)), movementSpeed(2.5f), mouseSensitivity(0.1f), zoom(45.0f) {
        position    = pos;
        worldUp     = up;
        this->yaw   = yaw;
        this->pitch = pitch;
        updateCameraVectors();
    }

    glm::mat4 GetViewMatrix() { return glm::lookAt(position, position + front, up); }

    void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true) {
        xoffset *= mouseSensitivity;
        yoffset *= mouseSensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (constrainPitch) {
            if (pitch > 89.0f)
                pitch = 89.0f;
            if (pitch < -89.0f)
                pitch = -89.0f;
        }

        updateCameraVectors();
    }

    void ProcessMouseScroll(float yoffset) {
        zoom -= (float)yoffset;
        if (zoom < 1.0f)
            zoom = 1.0f;
        if (zoom > 45.0f)
            zoom = 45.0f;
    }

    void Pan(float xoffset, float yoffset) {
        position -= right * xoffset * 0.01f;
        position += up * yoffset * 0.01f;
    }

   private:
    void updateCameraVectors() {
        glm::vec3 newFront;
        newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        newFront.y = sin(glm::radians(pitch));
        newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        front      = glm::normalize(newFront);

        right = glm::normalize(glm::cross(front, worldUp));
        up    = glm::normalize(glm::cross(right, front));
    }
};

// ========== OrbitCamera 类 (ADD) ==========
class OrbitCamera {
   public:
    float distance   = 3.0f;                // 相机距离
    float yaw        = 0.0f;                // 水平角
    float pitch      = 0.0f;                // 垂直角
    glm::vec3 target = glm::vec3(0, 0, 0);  // 相机观察的目标点

    glm::mat4 getViewMatrix() const {
        float x = distance * cosf(pitch) * cosf(yaw);
        float y = distance * cosf(pitch) * sinf(yaw);
        float z = distance * sinf(pitch);

        glm::vec3 eye = target + glm::vec3(x, y, z);
        return glm::lookAt(eye, target, glm::vec3(0, 0, 1));
    }
};

// =================== Robot (核心) ===================
class Robot {
   public:
    std::map<std::string, LinkVisual> visuals;
    std::map<std::string, glm::mat4> transforms;  // link_name -> global
    std::vector<JointState> joint_states;
    std::string package_path;                  // URDF 文件所在目录（你的原始实现）
    urdf::ModelInterfaceSharedPtr urdf_model;  // <<< MOD: 保存解析后的 model 指针
    std::string urdf_file_path;                // 保存路径（若需要）

    std::string resolvePath(const std::string& path) {
        if (path.rfind("package://", 0) == 0) {
            size_t package_end = path.find('/', 10);
            if (package_end != std::string::npos) {
                std::string package_name  = path.substr(10, package_end - 10);
                std::string relative_path = path.substr(package_end);

                if (!package_path.empty()) {
                    std::string candidate = package_path + relative_path;
                    std::ifstream f(candidate);
                    if (f.good()) {
                        return candidate;
                    }
                }

                std::string cmd = "rospack find " + package_name;
                FILE* pipe      = popen(cmd.c_str(), "r");
                if (!pipe)
                    return path;
                char buffer[512];
                std::string result;
                if (fgets(buffer, sizeof(buffer), pipe)) {
                    result = buffer;
                    if (!result.empty() && result.back() == '\n')
                        result.pop_back();
                }
                pclose(pipe);
                if (!result.empty()) {
                    std::string candidate = result + relative_path;
                    std::ifstream f2(candidate);
                    if (f2.good())
                        return candidate;
                }
            }
        }
        return path;
    }

    void loadURDF(const std::string& urdf_path) {
        urdf_file_path = urdf_path;
        size_t pos     = urdf_path.find_last_of('/');
        if (pos != std::string::npos) {
            package_path = urdf_path.substr(0, pos);
        }

        std::ifstream file(urdf_path);
        std::string xml_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml_str);
        if (!model) {
            std::cerr << "Failed to parse URDF file: " << urdf_path << std::endl;
            return;
        }
        urdf_model = model;

        initJointStates(model);

        visuals.clear();
        transforms.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse = [&](urdf::LinkConstSharedPtr link,
                                                                                       const glm::mat4& parent_transform) {
            transforms[link->name] = parent_transform;

            for (size_t i = 0; i < link->visual_array.size(); ++i) {
                auto visual = link->visual_array[i];
                if (!visual || !visual->geometry)
                    continue;

                LinkVisual lv;
                if (visual->geometry->type == urdf::Geometry::MESH) {
                    auto mesh    = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
                    lv.mesh_file = resolvePath(mesh->filename);
                    lv.scale     = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                } else {
                    continue;
                }

                auto origin   = visual->origin;
                lv.origin_xyz = glm::vec3(origin.position.x, origin.position.y, origin.position.z);
                double roll, pitch, yaw;
                origin.rotation.getRPY(roll, pitch, yaw);
                lv.origin_rpy = glm::vec3(roll, pitch, yaw);

                glm::mat4 localT    = glm::translate(glm::mat4(1.0f), lv.origin_xyz);
                localT              = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy.x, glm::vec3(1, 0, 0));
                localT              = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy.y, glm::vec3(0, 1, 0));
                localT              = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy.z, glm::vec3(0, 0, 1));
                lv.local_transform  = localT;
                lv.parent_link_name = link->name;

                if (!lv.mesh_file.empty()) {
                    lv.model.loadAssimp(lv.mesh_file);
                    lv.loaded = true;
                }

                std::string visual_name = link->name;
                if (link->visual_array.size() > 1) {
                    visual_name += "_visual_" + std::to_string(i);
                }
                visuals[visual_name] = lv;
            }

            for (auto& child_link : link->child_links) {
                auto joint                = child_link->parent_joint;
                glm::mat4 joint_transform = parent_transform;

                if (joint) {
                    auto joint_origin   = joint->parent_to_joint_origin_transform;
                    glm::vec3 joint_xyz = glm::vec3(joint_origin.position.x, joint_origin.position.y, joint_origin.position.z);
                    double rr, pp, yy;
                    joint_origin.rotation.getRPY(rr, pp, yy);
                    glm::mat4 joint_offset = glm::translate(glm::mat4(1.0f), joint_xyz) *
                                             glm::rotate(glm::mat4(1.0f), (float)rr, glm::vec3(1, 0, 0)) *
                                             glm::rotate(glm::mat4(1.0f), (float)pp, glm::vec3(0, 1, 0)) *
                                             glm::rotate(glm::mat4(1.0f), (float)yy, glm::vec3(0, 0, 1));

                    joint_transform = joint_transform * joint_offset;

                    for (auto& js : joint_states) {
                        if (js.name == joint->name) {
                            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                if (glm::length(axis) > 0.0001f)
                                    axis = glm::normalize(axis);
                                else
                                    axis = glm::vec3(0, 0, 1);
                                joint_transform = joint_transform * glm::rotate(glm::mat4(1.0f), js.position, axis);
                            } else if (joint->type == urdf::Joint::PRISMATIC) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                if (glm::length(axis) > 0.0001f)
                                    axis = glm::normalize(axis);
                                else
                                    axis = glm::vec3(1, 0, 0);
                                glm::vec3 translation = axis * js.position;
                                joint_transform       = joint_transform * glm::translate(glm::mat4(1.0f), translation);
                            }
                            break;
                        }
                    }
                }

                traverse(child_link, joint_transform);
            }
        };

        traverse(model->getRoot(), glm::mat4(1.0f));
    }

    void initJointStates(urdf::ModelInterfaceSharedPtr model) {
        joint_states.clear();
        std::function<void(urdf::LinkConstSharedPtr)> collectJoints = [&](urdf::LinkConstSharedPtr link) {
            for (auto& child_link : link->child_links) {
                auto joint = child_link->parent_joint;
                if (joint) {
                    JointState js;
                    js.name      = joint->name;
                    js.position  = 0.0f;
                    js.min_angle = joint->limits ? static_cast<float>(joint->limits->lower) : -3.14f;
                    js.max_angle = joint->limits ? static_cast<float>(joint->limits->upper) : 3.14f;
                    joint_states.push_back(js);
                }
                collectJoints(child_link);
            }
        };
        collectJoints(model->getRoot());
    }

    void draw(GLuint shader, const glm::mat4& view, const glm::mat4& proj) {
        for (auto& [visual_key, lv] : visuals) {
            if (!lv.loaded)
                continue;

            auto it = transforms.find(lv.parent_link_name);
            if (it == transforms.end())
                continue;

            glm::mat4 link_global = it->second;

            // <<< MOD: 保证 visual origin 叠加到 link global
            glm::mat4 model_mat = link_global * lv.local_transform;
            model_mat           = model_mat * glm::scale(glm::mat4(1.0f), lv.scale);

            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
            for (auto& mesh : lv.model.meshes) {
                mesh.draw(shader);
            }
        }
    }

    void updateTransforms() {
        if (!urdf_model)
            return;

        transforms.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse;
        traverse = [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform) {
            // 保存当前 link 的 global transform
            transforms[link->name] = parent_transform;

            // 遍历子 link
            for (auto& child_link : link->child_links) {
                auto joint = child_link->parent_joint;
                if (!joint)
                    continue;

                // === joint->origin (xyz + rpy) ===
                urdf::Vector3 p  = joint->parent_to_joint_origin_transform.position;
                urdf::Rotation r = joint->parent_to_joint_origin_transform.rotation;
                double roll, pitch, yaw;
                r.getRPY(roll, pitch, yaw);

                glm::mat4 joint_origin = glm::translate(glm::mat4(1.0f), glm::vec3(p.x, p.y, p.z)) *
                                         glm::mat4_cast(glm::quat(glm::vec3((float)roll, (float)pitch, (float)yaw)));

                // === joint 动态 transform (revolute / prismatic) ===
                glm::mat4 joint_motion(1.0f);
                for (auto& js : joint_states) {
                    if (js.name == joint->name) {
                        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                            glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                            if (glm::length(axis) < 1e-6f)
                                axis = glm::vec3(0, 0, 1);
                            joint_motion = glm::rotate(glm::mat4(1.0f), js.position, glm::normalize(axis));
                        } else if (joint->type == urdf::Joint::PRISMATIC) {
                            glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                            if (glm::length(axis) < 1e-6f)
                                axis = glm::vec3(1, 0, 0);
                            joint_motion = glm::translate(glm::mat4(1.0f), js.position * glm::normalize(axis));
                        }
                        break;
                    }
                }

                // === 子 link 的最终变换 ===
                glm::mat4 child_transform = parent_transform * joint_origin * joint_motion;

                // 递归
                traverse(child_link, child_transform);
            }
        };

        // 从 root 开始
        traverse(urdf_model->getRoot(), glm::mat4(1.0f));
    }

    void updateJointTransform(const std::string& joint_name, float new_position) {
        for (auto& js : joint_states) {
            if (js.name == joint_name) {
                js.position = new_position;
                break;
            }
        }
        updateTransforms();
    }
};

// ======================== 全局 / 回调（保持原样） ========================
// Camera camera;
// bool firstMouse = true;
// float lastX = 400, lastY = 300;
// bool mouseLeftPressed   = false;
// bool mouseMiddlePressed = false;

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    // 你原来注释掉了鼠标操作；保持不动（若需要启用，把下面的注释打开）
}
// void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
//     // camera.ProcessMouseScroll(yoffset);
// }
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    // 保持你原来的注释
}

// ========== 相机交互状态 (ADD) ==========
static OrbitCamera camera;
static double lastX = 0.0, lastY = 0.0;
static bool rotating       = false;
static double scrollOffset = 0.0;

// GLFW 滚轮回调
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    scrollOffset = yoffset;
}

// ======================== main (保留你原始实现，略作小修) ========================
int main() {
    // 初始化 GLFW
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Robot URDF Viewer with ImGui", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // 设置回调
    glfwSetCursorPosCallback(window, mouse_callback);
    // glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    // ========== 注册滚轮回调 (ADD) ==========
    glfwSetScrollCallback(window, scroll_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to init GLAD" << std::endl;
        return -1;
    }

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 编译着色器（保持你原来的）
    GLuint shader = glCreateProgram();
    GLuint vs     = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertex_shader_src, nullptr);
    glCompileShader(vs);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragment_shader_src, nullptr);
    glCompileShader(fs);
    glAttachShader(shader, vs);
    glAttachShader(shader, fs);
    glLinkProgram(shader);

    glEnable(GL_DEPTH_TEST);

    // 加载机器人（改为传入你自己的路径）
    Robot robot;
    robot.loadURDF(
        "/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/galbot_description/galbot_one_charlie_description/"
        "galbot_one_charlie.urdf");

    // 主循环
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (!io.WantCaptureMouse) {
            // 鼠标右键拖动旋转相机
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                if (!rotating) {
                    rotating = true;
                    lastX    = xpos;
                    lastY    = ypos;
                } else {
                    double dx = xpos - lastX;
                    double dy = ypos - lastY;
                    lastX     = xpos;
                    lastY     = ypos;

                    camera.yaw += 0.005f * (float)dx;
                    camera.pitch += 0.005f * (float)dy;
                    if (camera.pitch > 1.5f)
                        camera.pitch = 1.5f;
                    if (camera.pitch < -1.5f)
                        camera.pitch = -1.5f;
                }
            } else {
                rotating = false;
            }
        }

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        int side_panel_width = 300;
        int render_width     = w - side_panel_width;
        int render_height    = h;

        glViewport(0, 0, render_width, render_height);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3D 渲染
        glUseProgram(shader);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)render_width / (float)render_height, 0.01f, 100.0f);
        glm::mat4 view       = camera.getViewMatrix();
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));

        // glm::vec3 lightPos = camera.position + camera.front * 5.0f;
        // glUniform3f(glGetUniformLocation(shader, "lightPos"), lightPos.x, lightPos.y, lightPos.z);
        // glUniform3f(glGetUniformLocation(shader, "viewPos"), camera.position.x, camera.position.y, camera.position.z);

        // 在 draw 之前确保 transforms 是最新的
        robot.updateTransforms();
        robot.draw(shader, view, projection);

        // ImGui 界面
        glViewport(w - side_panel_width, 0, side_panel_width, h);
        glDisable(GL_DEPTH_TEST);
        ImGui::SetNextWindowPos(ImVec2(w - side_panel_width, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(side_panel_width, h));

        ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Robot Control", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

        ImGui::Text("Robot Joint Control");
        ImGui::Separator();

        for (auto& js : robot.joint_states) {
            if (robot.urdf_model->getJoint(js.name)->type != urdf::Joint::REVOLUTE) {
                continue;
            }

            float old_position = js.position;
            // <<< MOD: 将 joint limits（radian）转换为 degree 传给 SliderAngle 的 min/max
            ImGui::SliderAngle(js.name.c_str(), &js.position, glm::degrees(js.min_angle), glm::degrees(js.max_angle));
            if (old_position != js.position) {
                // 变动时只重新计算变换（updateTransforms 已经只做重算）
                robot.updateTransforms();
            }
        }

        ImGui::Text("Camera Controls:");
        ImGui::Text("Right Mouse Button + Drag: Rotate");
        ImGui::Text("Mouse Wheel: Zoom");
        // ImGui::Text("Current Position: (%.2f, %.2f, %.2f)", camera.position.x, camera.position.y, camera.position.z);
        // ImGui::Text("Yaw: %.2f, Pitch: %.2f", camera.yaw, camera.pitch);

        ImGui::End();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glViewport(0, 0, w, h);
        glEnable(GL_DEPTH_TEST);

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
