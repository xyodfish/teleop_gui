#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "teleop_viewer/scene.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

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
#include <set>
#include <unordered_map>
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
            unsigned int vao        = 0;
            unsigned int vbo        = 0;
            unsigned int ebo        = 0;

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

            void draw(GLuint shader, const glm::vec3* override_color = nullptr, bool force_color_only = false) {
                glBindVertexArray(vao);
                const glm::vec3 color = override_color ? *override_color : diffuse_color;
                glUniform3f(glGetUniformLocation(shader, "diffuseColor"), color.r, color.g, color.b);
                if (!textures.empty() && !force_color_only) {
                    glUniform1i(glGetUniformLocation(shader, "hasTexture"), true);
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D, textures[0].id);
                } else {
                    glUniform1i(glGetUniformLocation(shader, "hasTexture"), false);
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

                int width           = 0;
                int height          = 0;
                int nr_components   = 0;
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
            glm::vec3 urdf_color      = glm::vec3(0.8f, 0.8f, 0.8f);
            bool has_urdf_color       = false;

            Model model;
            bool loaded                     = false;
            glm::vec3 local_bounding_center = glm::vec3(0.0f);
            float local_bounding_radius     = 0.0f;
        };

        struct LocalCollisionProxy {
            std::string link_name;
            std::string collision_name;
            glm::mat4 local_transform = glm::mat4(1.0f);
            glm::vec3 local_center    = glm::vec3(0.0f);
            float radius_m            = 0.0f;
        };

        struct JointState {
            std::string name;
            float position  = 0.0f;
            float min_angle = -3.14f;
            float max_angle = 3.14f;
            bool revolute   = false;
        };

        struct JointAxisState {
            std::string name;
            glm::vec3 axis_local   = glm::vec3(0.0f, 0.0f, 1.0f);
            glm::vec3 world_origin = glm::vec3(0.0f);
            glm::vec3 world_axis   = glm::vec3(0.0f, 0.0f, 1.0f);
            bool revolute          = false;
        };

        glm::mat4 poseToTransform(const urdf::Pose& pose) {
            glm::vec3 origin_xyz(pose.position.x, pose.position.y, pose.position.z);
            double roll  = 0.0;
            double pitch = 0.0;
            double yaw   = 0.0;
            pose.rotation.getRPY(roll, pitch, yaw);
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), origin_xyz);
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(roll), glm::vec3(1, 0, 0));
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(pitch), glm::vec3(0, 1, 0));
            transform           = transform * glm::rotate(glm::mat4(1.0f), static_cast<float>(yaw), glm::vec3(0, 0, 1));
            return transform;
        }

    }  // namespace

    struct RobotScene::Impl {
        std::map<std::string, LinkVisual> visuals;
        std::vector<LocalCollisionProxy> collision_proxies_local;
        std::map<std::string, glm::mat4> transforms;
        std::map<std::string, std::string> link_parent;
        std::vector<JointState> joint_states;
        std::vector<JointAxisState> joint_axis_states;
        std::unordered_map<std::string, size_t> joint_axis_index;

        std::string package_path;
        std::string urdf_file_path;
        urdf::ModelInterfaceSharedPtr urdf_model;

        bool fixed_base_mode       = true;
        float virtual_base_x_m     = 0.0f;
        float virtual_base_y_m     = 0.0f;
        float virtual_base_yaw_rad = 0.0f;

        glm::mat4 rootWorldTransform() const {
            if (fixed_base_mode) {
                return glm::mat4(1.0f);
            }
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(virtual_base_x_m, virtual_base_y_m, 0.0f));
            transform           = transform * glm::rotate(glm::mat4(1.0f), virtual_base_yaw_rad, glm::vec3(0.0f, 0.0f, 1.0f));
            return transform;
        }

        bool computeBoundingSphereFromModel(const Model& model, glm::vec3* out_center, float* out_radius) const {
            if (out_center == nullptr || out_radius == nullptr) {
                return false;
            }
            glm::vec3 min_v(1e9f);
            glm::vec3 max_v(-1e9f);
            bool has_vertex = false;
            for (const auto& mesh : model.meshes) {
                for (const auto& vertex : mesh.vertices) {
                    min_v      = glm::min(min_v, vertex.position);
                    max_v      = glm::max(max_v, vertex.position);
                    has_vertex = true;
                }
            }
            if (!has_vertex) {
                *out_center = glm::vec3(0.0f);
                *out_radius = 0.0f;
                return false;
            }

            const glm::vec3 center = 0.5f * (min_v + max_v);
            float radius           = 0.0f;
            for (const auto& mesh : model.meshes) {
                for (const auto& vertex : mesh.vertices) {
                    radius = std::max(radius, glm::length(vertex.position - center));
                }
            }
            *out_center = center;
            *out_radius = radius;
            return true;
        }

        void computeBoundingSphere(LinkVisual* visual) const {
            if (visual == nullptr) {
                return;
            }
            if (!computeBoundingSphereFromModel(visual->model, &visual->local_bounding_center, &visual->local_bounding_radius)) {
                visual->local_bounding_center = glm::vec3(0.0f);
                visual->local_bounding_radius = 0.0f;
            }
        }

        bool buildCollisionProxyFromGeometry(const urdf::CollisionSharedPtr& collision, const std::string& link_name,
                                             const std::string& collision_name, LocalCollisionProxy* out_proxy) const {
            if (collision == nullptr || collision->geometry == nullptr || out_proxy == nullptr) {
                return false;
            }

            out_proxy->link_name       = link_name;
            out_proxy->collision_name  = collision_name;
            out_proxy->local_transform = poseToTransform(collision->origin);
            out_proxy->local_center    = glm::vec3(0.0f);
            out_proxy->radius_m        = 0.0f;

            if (collision->geometry->type == urdf::Geometry::SPHERE) {
                auto sphere         = std::static_pointer_cast<urdf::Sphere>(collision->geometry);
                out_proxy->radius_m = static_cast<float>(sphere->radius);
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::BOX) {
                auto box = std::static_pointer_cast<urdf::Box>(collision->geometry);
                const glm::vec3 dims(static_cast<float>(box->dim.x), static_cast<float>(box->dim.y), static_cast<float>(box->dim.z));
                out_proxy->radius_m = 0.5f * glm::length(dims);
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cylinder           = std::static_pointer_cast<urdf::Cylinder>(collision->geometry);
                const float radius      = static_cast<float>(cylinder->radius);
                const float half_length = static_cast<float>(0.5 * cylinder->length);
                out_proxy->radius_m     = std::sqrt(radius * radius + half_length * half_length);
                return out_proxy->radius_m > 1e-6f;
            }

            if (collision->geometry->type == urdf::Geometry::MESH) {
                auto mesh                   = std::static_pointer_cast<urdf::Mesh>(collision->geometry);
                const std::string mesh_file = resolvePath(mesh->filename);
                if (mesh_file.empty()) {
                    return false;
                }
                Model collision_model;
                collision_model.loadAssimp(mesh_file);
                glm::vec3 mesh_center(0.0f);
                float mesh_radius = 0.0f;
                if (!computeBoundingSphereFromModel(collision_model, &mesh_center, &mesh_radius)) {
                    return false;
                }
                const glm::vec3 mesh_scale(static_cast<float>(mesh->scale.x), static_cast<float>(mesh->scale.y),
                                           static_cast<float>(mesh->scale.z));
                out_proxy->local_transform = out_proxy->local_transform * glm::scale(glm::mat4(1.0f), mesh_scale);
                out_proxy->local_center    = mesh_center;
                const float scale_factor   = std::max(std::max(std::fabs(mesh_scale.x), std::fabs(mesh_scale.y)), std::fabs(mesh_scale.z));
                out_proxy->radius_m        = std::max(0.0f, mesh_radius * scale_factor);
                return out_proxy->radius_m > 1e-6f;
            }

            return false;
        }

        std::string resolvePath(const std::string& path) const {
            if (path.empty()) {
                return path;
            }

            // Absolute path: use as-is.
            if (path.front() == '/') {
                return path;
            }

            // Relative path from URDF directory.
            if (!package_path.empty()) {
                std::string candidate = package_path + "/" + path;
                std::ifstream f(candidate);
                if (f.good()) {
                    return candidate;
                }
            }

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
            joint_axis_states.clear();
            joint_axis_index.clear();
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

                        JointAxisState axis_state;
                        axis_state.name = joint->name;
                        glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                        if (glm::length(axis) < 1e-6f) {
                            axis = glm::vec3(0.0f, 0.0f, 1.0f);
                        }
                        axis_state.axis_local             = glm::normalize(axis);
                        axis_state.revolute               = js.revolute;
                        joint_axis_index[axis_state.name] = joint_axis_states.size();
                        joint_axis_states.push_back(axis_state);
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
        impl_->collision_proxies_local.clear();
        impl_->transforms.clear();
        impl_->link_parent.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&, const std::string&)> traverse =
            [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform, const std::string& parent_name) {
                impl_->transforms[link->name]  = parent_transform;
                impl_->link_parent[link->name] = parent_name;

                for (size_t i = 0; i < link->visual_array.size(); ++i) {
                    auto visual = link->visual_array[i];
                    if (!visual || !visual->geometry) {
                        continue;
                    }

                    LinkVisual lv;
                    if (visual->geometry->type == urdf::Geometry::MESH) {
                        auto mesh    = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
                        lv.mesh_file = impl_->resolvePath(mesh->filename);
                        lv.scale     = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                    } else {
                        continue;
                    }

                    lv.local_transform  = poseToTransform(visual->origin);
                    lv.parent_link_name = link->name;
                    if (visual->material) {
                        lv.urdf_color =
                            glm::vec3(static_cast<float>(visual->material->color.r), static_cast<float>(visual->material->color.g),
                                      static_cast<float>(visual->material->color.b));
                        lv.has_urdf_color = true;
                    }

                    if (!lv.mesh_file.empty()) {
                        lv.model.loadAssimp(lv.mesh_file);
                        lv.loaded = true;
                        impl_->computeBoundingSphere(&lv);
                    }

                    std::string visual_name = link->name;
                    if (link->visual_array.size() > 1) {
                        visual_name += "_visual_" + std::to_string(i);
                    }
                    impl_->visuals[visual_name] = lv;
                }

                std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
                if (collisions.empty() && link->collision != nullptr) {
                    collisions.push_back(link->collision);
                }
                for (size_t i = 0; i < collisions.size(); ++i) {
                    const auto& collision = collisions[i];
                    if (collision == nullptr || collision->geometry == nullptr) {
                        continue;
                    }
                    std::string collision_name = collision->name;
                    if (collision_name.empty()) {
                        collision_name = link->name + "_collision_" + std::to_string(i);
                    }
                    LocalCollisionProxy local_proxy;
                    if (!impl_->buildCollisionProxyFromGeometry(collision, link->name, collision_name, &local_proxy)) {
                        continue;
                    }
                    impl_->collision_proxies_local.push_back(std::move(local_proxy));
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
                                axis            = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(0, 0, 1);
                                joint_transform = joint_transform * glm::rotate(glm::mat4(1.0f), js.position, axis);
                            } else if (joint->type == urdf::Joint::PRISMATIC) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                axis                  = glm::length(axis) > 0.0001f ? glm::normalize(axis) : glm::vec3(1, 0, 0);
                                glm::vec3 translation = axis * js.position;
                                joint_transform       = joint_transform * glm::translate(glm::mat4(1.0f), translation);
                            }
                            break;
                        }
                    }

                    traverse(child_link, joint_transform, link->name);
                }
            };

        traverse(model->getRoot(), impl_->rootWorldTransform(), "");
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
                double roll      = 0.0;
                double pitch     = 0.0;
                double yaw       = 0.0;
                r.getRPY(roll, pitch, yaw);

                glm::mat4 joint_origin = glm::translate(glm::mat4(1.0f), glm::vec3(p.x, p.y, p.z)) *
                                         glm::mat4_cast(glm::quat(glm::vec3((float)roll, (float)pitch, (float)yaw)));

                glm::mat4 joint_motion(1.0f);

                glm::mat4 joint_frame_world = parent_transform * joint_origin;
                auto axis_it                = impl_->joint_axis_index.find(joint->name);
                if (axis_it != impl_->joint_axis_index.end()) {
                    JointAxisState& axis_state = impl_->joint_axis_states[axis_it->second];
                    axis_state.world_origin    = glm::vec3(joint_frame_world[3]);
                    glm::vec3 axis_world       = glm::mat3(joint_frame_world) * axis_state.axis_local;
                    if (glm::length(axis_world) < 1e-6f) {
                        axis_world = glm::vec3(0.0f, 0.0f, 1.0f);
                    }
                    axis_state.world_axis = glm::normalize(axis_world);
                }

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

        traverse(impl_->urdf_model->getRoot(), impl_->rootWorldTransform());
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
                if (lv.has_urdf_color) {
                    mesh.draw(shader, &lv.urdf_color, true);
                } else {
                    mesh.draw(shader);
                }
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

    std::vector<RobotScene::JointAxisInfo> RobotScene::getJointAxisInfos(bool revolute_only) const {
        std::vector<JointAxisInfo> infos;
        infos.reserve(impl_->joint_axis_states.size());

        for (const auto& axis_state : impl_->joint_axis_states) {
            if (revolute_only && !axis_state.revolute) {
                continue;
            }
            JointAxisInfo info;
            info.name         = axis_state.name;
            info.world_origin = axis_state.world_origin;
            info.world_axis   = axis_state.world_axis;
            info.revolute     = axis_state.revolute;
            infos.push_back(std::move(info));
        }

        return infos;
    }

    std::vector<RobotScene::LinkTfInfo> RobotScene::getLinkTfInfos() const {
        std::vector<LinkTfInfo> infos;
        infos.reserve(impl_->transforms.size());

        for (const auto& [link_name, tf] : impl_->transforms) {
            LinkTfInfo info;
            info.name      = link_name;
            auto parent_it = impl_->link_parent.find(link_name);
            if (parent_it != impl_->link_parent.end()) {
                info.parent_name = parent_it->second;
            }
            info.world_position = glm::vec3(tf[3]);

            glm::quat q     = glm::quat_cast(tf);
            glm::vec3 euler = glm::eulerAngles(q);
            info.world_rpy  = euler;
            infos.push_back(std::move(info));
        }

        return infos;
    }

    std::vector<RobotScene::LinkCollisionProxy> RobotScene::getLinkCollisionProxies() const {
        std::vector<LinkCollisionProxy> proxies;
        if (!impl_->collision_proxies_local.empty()) {
            proxies.reserve(impl_->collision_proxies_local.size());
            for (const auto& local_proxy : impl_->collision_proxies_local) {
                if (local_proxy.radius_m <= 1e-6f) {
                    continue;
                }
                const auto link_it = impl_->transforms.find(local_proxy.link_name);
                if (link_it == impl_->transforms.end()) {
                    continue;
                }

                const glm::mat4 world_transform = link_it->second * local_proxy.local_transform;
                LinkCollisionProxy proxy;
                proxy.link_name    = local_proxy.link_name;
                proxy.visual_name  = local_proxy.collision_name;
                proxy.world_center = glm::vec3(world_transform * glm::vec4(local_proxy.local_center, 1.0f));
                proxy.radius_m     = local_proxy.radius_m;
                proxies.push_back(std::move(proxy));
            }
            return proxies;
        }

        proxies.reserve(impl_->visuals.size());
        for (const auto& [visual_name, visual] : impl_->visuals) {
            if (!visual.loaded || visual.local_bounding_radius <= 1e-6f) {
                continue;
            }

            const auto link_it = impl_->transforms.find(visual.parent_link_name);
            if (link_it == impl_->transforms.end()) {
                continue;
            }

            glm::mat4 proxy_transform    = link_it->second * visual.local_transform;
            proxy_transform              = proxy_transform * glm::scale(glm::mat4(1.0f), visual.scale);
            const glm::vec3 world_center = glm::vec3(proxy_transform * glm::vec4(visual.local_bounding_center, 1.0f));
            const float scale_factor = std::max(std::max(std::fabs(visual.scale.x), std::fabs(visual.scale.y)), std::fabs(visual.scale.z));

            LinkCollisionProxy proxy;
            proxy.link_name    = visual.parent_link_name;
            proxy.visual_name  = visual_name;
            proxy.world_center = world_center;
            proxy.radius_m     = std::max(0.0f, visual.local_bounding_radius * scale_factor);
            proxies.push_back(std::move(proxy));
        }
        return proxies;
    }

    bool RobotScene::getLinkWorldTransform(const std::string& link_name, glm::mat4* out_world_transform) const {
        if (out_world_transform == nullptr) {
            return false;
        }
        const auto it = impl_->transforms.find(link_name);
        if (it == impl_->transforms.end()) {
            return false;
        }
        *out_world_transform = it->second;
        return true;
    }

    bool RobotScene::getLinkParentName(const std::string& link_name, std::string* out_parent_name) const {
        if (out_parent_name == nullptr) {
            return false;
        }
        auto it = impl_->link_parent.find(link_name);
        if (it == impl_->link_parent.end()) {
            return false;
        }
        *out_parent_name = it->second;
        return true;
    }

    void RobotScene::setFixedBaseMode(bool enabled) {
        impl_->fixed_base_mode = enabled;
    }
    bool RobotScene::fixedBaseMode() const {
        return impl_->fixed_base_mode;
    }

    void RobotScene::setVirtualBasePose2D(float x_m, float y_m, float yaw_rad) {
        impl_->virtual_base_x_m     = x_m;
        impl_->virtual_base_y_m     = y_m;
        impl_->virtual_base_yaw_rad = yaw_rad;
    }

    bool RobotScene::getVirtualBasePose2D(float* x_m, float* y_m, float* yaw_rad) const {
        if (x_m == nullptr || y_m == nullptr || yaw_rad == nullptr) {
            return false;
        }
        *x_m     = impl_->virtual_base_x_m;
        *y_m     = impl_->virtual_base_y_m;
        *yaw_rad = impl_->virtual_base_yaw_rad;
        return true;
    }

    glm::vec3 OrbitCamera::eye() const {
        float x = distance * cosf(pitch) * cosf(yaw);
        float y = distance * cosf(pitch) * sinf(yaw);
        float z = distance * sinf(pitch);
        return target + glm::vec3(x, y, z);
    }

    glm::mat4 OrbitCamera::viewMatrix() const {
        return glm::lookAt(eye(), target, glm::vec3(0, 0, 1));
    }

    void OrbitCamera::rotate(float dx, float dy) {
        yaw += rotate_speed * dx;
        pitch += rotate_speed * dy;
        pitch = std::clamp(pitch, -1.55f, 1.55f);
    }

    void OrbitCamera::zoom(float delta) {
        distance *= (1.0f - zoom_scale * delta);
        distance = std::clamp(distance, min_distance, max_distance);
    }

    void OrbitCamera::dolly(float dy) {
        zoom(dolly_scale * dy);
    }

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
