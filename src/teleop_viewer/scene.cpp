#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "teleop_viewer/scene.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <urdf_parser/urdf_parser.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <utility>

namespace omnilink::teleop_viewer {
namespace {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 tex_coords;
};

struct Texture {
    unsigned int id = 0;
    std::string type;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    glm::vec3 diffuse_color = glm::vec3(0.8f, 0.8f, 0.8f);
    unsigned int vao = 0;
    unsigned int vbo = 0;
    unsigned int ebo = 0;

    void setup() {
        if (vao) {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
            glDeleteBuffers(1, &ebo);
            vao = vbo = ebo = 0;
        }

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tex_coords));

        glBindVertexArray(0);
    }

    void draw(GLuint shader) {
        glBindVertexArray(vao);
        if (!textures.empty()) {
            glUniform1i(glGetUniformLocation(shader, "hasTexture"), true);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, textures[0].id);
        } else {
            glUniform1i(glGetUniformLocation(shader, "hasTexture"), false);
            glUniform3f(glGetUniformLocation(shader, "diffuseColor"), diffuse_color.r, diffuse_color.g, diffuse_color.b);
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

        size_t last_slash = path.find_last_of('/');
        if (last_slash != std::string::npos) {
            directory = path.substr(0, last_slash);
        }

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
            vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);

            if (mesh->HasNormals()) {
                vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
            } else {
                vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
            }

            if (mesh->mTextureCoords[0]) {
                vertex.tex_coords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            } else {
                vertex.tex_coords = glm::vec2(0.0f, 0.0f);
            }

            m.vertices.push_back(vertex);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; j++) {
                m.indices.push_back(face.mIndices[j]);
            }
        }

        if (mesh->mMaterialIndex >= 0) {
            aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
            aiColor3D color(1.0f, 1.0f, 1.0f);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
            m.diffuse_color = glm::vec3(color.r, color.g, color.b);

            if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
                aiString str;
                material->GetTexture(aiTextureType_DIFFUSE, 0, &str);
                std::string tex_path = directory + "/" + str.C_Str();
                Texture tex;
                tex.id   = loadTexture(tex_path);
                tex.type = "texture_diffuse";
                m.textures.push_back(tex);
            }
        }

        m.setup();
        return m;
    }

    unsigned int loadTexture(const std::string& path) {
        unsigned int texture_id;
        glGenTextures(1, &texture_id);

        int width = 0;
        int height = 0;
        int nr_components = 0;
        unsigned char* data = stbi_load(path.c_str(), &width, &height, &nr_components, 0);

        if (data) {
            GLenum format = nr_components == 1 ? GL_RED : nr_components == 3 ? GL_RGB : GL_RGBA;
            glBindTexture(GL_TEXTURE_2D, texture_id);
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

        return texture_id;
    }
};

struct LinkVisual {
    std::string mesh_file;
    glm::vec3 scale = glm::vec3(1.0f);

    std::string parent_link_name;
    glm::mat4 local_transform = glm::mat4(1.0f);

    Model model;
    bool loaded = false;
};

struct JointState {
    std::string name;
    float position = 0.0f;
    float min_angle = -3.14f;
    float max_angle = 3.14f;
    bool revolute = false;
};

}  // namespace

struct RobotScene::Impl {
    std::map<std::string, LinkVisual> visuals;
    std::map<std::string, glm::mat4> transforms;
    std::vector<JointState> joint_states;

    std::string package_path;
    std::string urdf_file_path;
    urdf::ModelInterfaceSharedPtr urdf_model;

    bool fixed_base_mode = true;

    std::string resolvePath(const std::string& path) const {
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
                if (!pipe) {
                    return path;
                }

                char buffer[512];
                std::string result;
                if (fgets(buffer, sizeof(buffer), pipe)) {
                    result = buffer;
                    if (!result.empty() && result.back() == '\n') {
                        result.pop_back();
                    }
                }
                pclose(pipe);

                if (!result.empty()) {
                    std::string candidate = result + relative_path;
                    std::ifstream f2(candidate);
                    if (f2.good()) {
                        return candidate;
                    }
                }
            }
        }
        return path;
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
                    js.revolute  = (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS);
                    joint_states.push_back(js);
                }
                collectJoints(child_link);
            }
        };
        collectJoints(model->getRoot());
    }
};

RobotScene::RobotScene() : impl_(std::make_unique<Impl>()) {}
RobotScene::~RobotScene() = default;

bool RobotScene::loadURDF(const std::string& urdf_path) {
    impl_->urdf_file_path = urdf_path;
    size_t pos            = urdf_path.find_last_of('/');
    if (pos != std::string::npos) {
        impl_->package_path = urdf_path.substr(0, pos);
    }

    std::ifstream file(urdf_path);
    if (!file.good()) {
        std::cerr << "Failed to open URDF file: " << urdf_path << std::endl;
        return false;
    }

    std::string xml_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto model = urdf::parseURDF(xml_str);
    if (!model) {
        std::cerr << "Failed to parse URDF file: " << urdf_path << std::endl;
        return false;
    }

    impl_->urdf_model = model;
    impl_->initJointStates(model);

    impl_->visuals.clear();
    impl_->transforms.clear();

    std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse = [&](urdf::LinkConstSharedPtr link,
                                                                                     const glm::mat4& parent_transform) {
        impl_->transforms[link->name] = parent_transform;

        for (size_t i = 0; i < link->visual_array.size(); ++i) {
            auto visual = link->visual_array[i];
            if (!visual || !visual->geometry) {
                continue;
            }

            LinkVisual lv;
            if (visual->geometry->type == urdf::Geometry::MESH) {
                auto mesh     = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
                lv.mesh_file  = impl_->resolvePath(mesh->filename);
                lv.scale      = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
            } else {
                continue;
            }

            auto origin = visual->origin;
            glm::vec3 origin_xyz(origin.position.x, origin.position.y, origin.position.z);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            origin.rotation.getRPY(roll, pitch, yaw);

            glm::mat4 localT = glm::translate(glm::mat4(1.0f), origin_xyz);
            localT = localT * glm::rotate(glm::mat4(1.0f), static_cast<float>(roll), glm::vec3(1, 0, 0));
            localT = localT * glm::rotate(glm::mat4(1.0f), static_cast<float>(pitch), glm::vec3(0, 1, 0));
            localT = localT * glm::rotate(glm::mat4(1.0f), static_cast<float>(yaw), glm::vec3(0, 0, 1));

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
            impl_->visuals[visual_name] = lv;
        }

        for (auto& child_link : link->child_links) {
            auto joint                = child_link->parent_joint;
            glm::mat4 joint_transform = parent_transform;

            if (joint) {
                auto joint_origin   = joint->parent_to_joint_origin_transform;
                glm::vec3 joint_xyz = glm::vec3(joint_origin.position.x, joint_origin.position.y, joint_origin.position.z);

                double rr = 0.0;
                double pp = 0.0;
                double yy = 0.0;
                joint_origin.rotation.getRPY(rr, pp, yy);

                glm::mat4 joint_offset = glm::translate(glm::mat4(1.0f), joint_xyz) *
                                         glm::rotate(glm::mat4(1.0f), static_cast<float>(rr), glm::vec3(1, 0, 0)) *
                                         glm::rotate(glm::mat4(1.0f), static_cast<float>(pp), glm::vec3(0, 1, 0)) *
                                         glm::rotate(glm::mat4(1.0f), static_cast<float>(yy), glm::vec3(0, 0, 1));

                joint_transform = joint_transform * joint_offset;

                for (const auto& js : impl_->joint_states) {
                    if (js.name != joint->name) {
                        continue;
                    }

                    if (impl_->fixed_base_mode && IsBaseMotionJointName(joint->name)) {
                        break;
                    }

                    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        axis = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(0, 0, 1);
                        joint_transform = joint_transform * glm::rotate(glm::mat4(1.0f), js.position, axis);
                    } else if (joint->type == urdf::Joint::PRISMATIC) {
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        axis = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(1, 0, 0);
                        glm::vec3 translation = axis * js.position;
                        joint_transform       = joint_transform * glm::translate(glm::mat4(1.0f), translation);
                    }
                    break;
                }
            }

            traverse(child_link, joint_transform);
        }
    };

    traverse(model->getRoot(), glm::mat4(1.0f));
    return true;
}

void RobotScene::updateTransforms() {
    if (!impl_->urdf_model) {
        return;
    }

    impl_->transforms.clear();

    std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse;
    traverse = [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform) {
        impl_->transforms[link->name] = parent_transform;

        for (auto& child_link : link->child_links) {
            auto joint = child_link->parent_joint;
            if (!joint) {
                continue;
            }

            urdf::Vector3 p  = joint->parent_to_joint_origin_transform.position;
            urdf::Rotation r = joint->parent_to_joint_origin_transform.rotation;
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            r.getRPY(roll, pitch, yaw);

            glm::mat4 joint_origin = glm::translate(glm::mat4(1.0f), glm::vec3(p.x, p.y, p.z)) *
                                     glm::mat4_cast(glm::quat(glm::vec3((float)roll, (float)pitch, (float)yaw)));

            glm::mat4 joint_motion(1.0f);
            for (const auto& js : impl_->joint_states) {
                if (js.name != joint->name) {
                    continue;
                }

                if (impl_->fixed_base_mode && IsBaseMotionJointName(joint->name)) {
                    break;
                }

                if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                    glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                    if (glm::length(axis) < 1e-6f) {
                        axis = glm::vec3(0, 0, 1);
                    }
                    joint_motion = glm::rotate(glm::mat4(1.0f), js.position, glm::normalize(axis));
                } else if (joint->type == urdf::Joint::PRISMATIC) {
                    glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                    if (glm::length(axis) < 1e-6f) {
                        axis = glm::vec3(1, 0, 0);
                    }
                    joint_motion = glm::translate(glm::mat4(1.0f), js.position * glm::normalize(axis));
                }
                break;
            }

            glm::mat4 child_transform = parent_transform * joint_origin * joint_motion;
            traverse(child_link, child_transform);
        }
    };

    traverse(impl_->urdf_model->getRoot(), glm::mat4(1.0f));
}

void RobotScene::draw(GLuint shader) {
    for (auto& [visual_key, lv] : impl_->visuals) {
        (void)visual_key;
        if (!lv.loaded) {
            continue;
        }

        auto it = impl_->transforms.find(lv.parent_link_name);
        if (it == impl_->transforms.end()) {
            continue;
        }

        glm::mat4 link_global = it->second;
        glm::mat4 model_mat   = link_global * lv.local_transform;
        model_mat             = model_mat * glm::scale(glm::mat4(1.0f), lv.scale);

        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
        for (auto& mesh : lv.model.meshes) {
            mesh.draw(shader);
        }
    }
}

size_t RobotScene::applyJointSamples(const std::vector<SensorJointSample>& samples, bool only_master_arm) {
    size_t applied = 0;
    for (const auto& sample : samples) {
        if (!sample.has_position || !std::isfinite(sample.position)) {
            continue;
        }
        if (only_master_arm && !IsMasterArmGroup(sample.group)) {
            continue;
        }

        if (setJointPositionByName(sample.name, static_cast<float>(sample.position))) {
            applied++;
        }
    }
    return applied;
}

bool RobotScene::setJointPositionByName(const std::string& joint_name, float new_position) {
    for (auto& js : impl_->joint_states) {
        if (js.name == joint_name) {
            js.position = new_position;
            return true;
        }
    }
    return false;
}

bool RobotScene::getJointInfo(const std::string& joint_name, JointInfo* out) const {
    if (!out) {
        return false;
    }

    for (const auto& js : impl_->joint_states) {
        if (js.name == joint_name) {
            out->name      = js.name;
            out->position  = js.position;
            out->min_angle = js.min_angle;
            out->max_angle = js.max_angle;
            out->revolute  = js.revolute;
            return true;
        }
    }
    return false;
}

std::vector<RobotScene::JointInfo> RobotScene::getJointInfos() const {
    std::vector<JointInfo> infos;
    infos.reserve(impl_->joint_states.size());

    for (const auto& js : impl_->joint_states) {
        JointInfo info;
        info.name      = js.name;
        info.position  = js.position;
        info.min_angle = js.min_angle;
        info.max_angle = js.max_angle;
        info.revolute  = js.revolute;
        infos.push_back(std::move(info));
    }

    return infos;
}

void RobotScene::setFixedBaseMode(bool enabled) { impl_->fixed_base_mode = enabled; }
bool RobotScene::fixedBaseMode() const { return impl_->fixed_base_mode; }

glm::vec3 OrbitCamera::eye() const {
    float x = distance * cosf(pitch) * cosf(yaw);
    float y = distance * cosf(pitch) * sinf(yaw);
    float z = distance * sinf(pitch);
    return target + glm::vec3(x, y, z);
}

glm::mat4 OrbitCamera::viewMatrix() const { return glm::lookAt(eye(), target, glm::vec3(0, 0, 1)); }

void OrbitCamera::rotate(float dx, float dy) {
    yaw += rotate_speed * dx;
    pitch += rotate_speed * dy;
    pitch = std::clamp(pitch, -1.55f, 1.55f);
}

void OrbitCamera::zoom(float delta) {
    distance *= (1.0f - zoom_scale * delta);
    distance = std::clamp(distance, min_distance, max_distance);
}

void OrbitCamera::dolly(float dy) { zoom(dolly_scale * dy); }

void OrbitCamera::pan(float dx, float dy) {
    glm::vec3 camera_eye = eye();
    glm::vec3 forward    = glm::normalize(target - camera_eye);
    glm::vec3 world_up(0.0f, 0.0f, 1.0f);

    glm::vec3 right = glm::cross(forward, world_up);
    if (glm::length(right) < 1e-6f) {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
        right = glm::normalize(right);
    }

    glm::vec3 up = glm::normalize(glm::cross(right, forward));
    target += (-right * dx + up * dy) * (pan_scale * distance);
}

}  // namespace omnilink::teleop_viewer
