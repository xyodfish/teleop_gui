#include "robot_loader.h"
#include <fstream>
#include <functional>

namespace omnilink::gui {
    std::string RobotLoader::resolvePath(const std::string& path) {
        if (path.rfind("package://", 0) == 0) {
            size_t package_end = path.find('/', 10);
            if (package_end != std::string::npos) {
                std::string package_name  = path.substr(10, package_end - 10);
                std::string relative_path = path.substr(package_end);

                if (!package_path_.empty()) {
                    std::string candidate = package_path_ + relative_path;
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

    void RobotLoader::loadURDF(const std::string& urdf_path) {
        urdf_file_path_ = urdf_path;
        size_t pos      = urdf_path.find_last_of('/');
        if (pos != std::string::npos) {
            package_path_ = urdf_path.substr(0, pos);
        }

        std::ifstream file(urdf_path);
        std::string xml_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml_str);
        if (!model) {
            std::cerr << "Failed to parse URDF file: " << urdf_path << std::endl;
            return;
        }
        urdf_model_ = model;

        initJointStates(model);

        visuals_.clear();
        transforms_.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse = [&](urdf::LinkConstSharedPtr link,
                                                                                       const glm::mat4& parent_transform) {
            transforms_[link->name] = parent_transform;

            for (size_t i = 0; i < link->visual_array.size(); ++i) {
                auto visual = link->visual_array[i];
                if (!visual || !visual->geometry)
                    continue;

                LinkVisual lv;
                if (visual->geometry->type == urdf::Geometry::MESH) {
                    auto mesh     = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
                    lv.mesh_file_ = resolvePath(mesh->filename);
                    lv.scale_     = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                } else {
                    continue;
                }

                auto origin    = visual->origin;
                lv.origin_xyz_ = glm::vec3(origin.position.x, origin.position.y, origin.position.z);
                double roll, pitch, yaw;
                origin.rotation.getRPY(roll, pitch, yaw);
                lv.origin_rpy_ = glm::vec3(roll, pitch, yaw);

                glm::mat4 localT     = glm::translate(glm::mat4(1.0f), lv.origin_xyz_);
                localT               = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy_.x, glm::vec3(1, 0, 0));
                localT               = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy_.y, glm::vec3(0, 1, 0));
                localT               = localT * glm::rotate(glm::mat4(1.0f), lv.origin_rpy_.z, glm::vec3(0, 0, 1));
                lv.local_transform_  = localT;
                lv.parent_link_name_ = link->name;

                if (!lv.mesh_file_.empty()) {
                    lv.model_.loadAssimp(lv.mesh_file_);
                    lv.loaded_ = true;
                }

                std::string visual_name = link->name;
                if (link->visual_array.size() > 1) {
                    visual_name += "_visual_" + std::to_string(i);
                }
                visuals_[visual_name] = lv;
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

                    for (auto& js : joint_states_) {
                        if (js.name_ == joint->name) {
                            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                if (glm::length(axis) > 0.0001f)
                                    axis = glm::normalize(axis);
                                else
                                    axis = glm::vec3(0, 0, 1);
                                joint_transform = joint_transform * glm::rotate(glm::mat4(1.0f), js.position_, axis);
                            } else if (joint->type == urdf::Joint::PRISMATIC) {
                                glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                                if (glm::length(axis) > 0.0001f)
                                    axis = glm::normalize(axis);
                                else
                                    axis = glm::vec3(1, 0, 0);
                                glm::vec3 translation = axis * js.position_;
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

    void Robot::initJointStates(urdf::ModelInterfaceSharedPtr model) {
        joint_states_.clear();
        std::function<void(urdf::LinkConstSharedPtr)> collectJoints = [&](urdf::LinkConstSharedPtr link) {
            for (auto& child_link : link->child_links) {
                auto joint = child_link->parent_joint;
                if (joint) {
                    JointState js;
                    js.name_      = joint->name;
                    js.position_  = 0.0f;
                    js.min_angle_ = joint->limits ? static_cast<float>(joint->limits->lower) : -3.14f;
                    js.max_angle_ = joint->limits ? static_cast<float>(joint->limits->upper) : 3.14f;
                    joint_states_.push_back(js);
                }
                collectJoints(child_link);
            }
        };
        collectJoints(model->getRoot());
    }

    void RobotLoader::draw(GLuint shader, const glm::mat4& view, const glm::mat4& proj) {
        for (auto& [visual_key, lv] : visuals_) {
            if (!lv.loaded_)
                continue;

            auto it = transforms_.find(lv.parent_link_name_);
            if (it == transforms_.end())
                continue;

            glm::mat4 link_global = it->second;

            // <<< MOD: 保证 visual origin 叠加到 link global
            glm::mat4 model_mat = link_global * lv.local_transform_;
            model_mat           = model_mat * glm::scale(glm::mat4(1.0f), lv.scale_);

            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
            for (auto& mesh : lv.model_.meshes_) {
                mesh.draw(shader);
            }
        }
    }

    void RobotLoader::updateTransforms() {
        if (!urdf_model_)
            return;

        transforms_.clear();

        std::function<void(urdf::LinkConstSharedPtr, const glm::mat4&)> traverse;
        traverse = [&](urdf::LinkConstSharedPtr link, const glm::mat4& parent_transform) {
            // 保存当前 link 的 global transform
            transforms_[link->name] = parent_transform;

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
                for (auto& js : joint_states_) {
                    if (js.name_ == joint->name) {
                        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                            glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                            if (glm::length(axis) < 1e-6f)
                                axis = glm::vec3(0, 0, 1);
                            joint_motion = glm::rotate(glm::mat4(1.0f), js.position_, glm::normalize(axis));
                        } else if (joint->type == urdf::Joint::PRISMATIC) {
                            glm::vec3 axis(joint->axis.x, joint->axis.y, joint->axis.z);
                            if (glm::length(axis) < 1e-6f)
                                axis = glm::vec3(1, 0, 0);
                            joint_motion = glm::translate(glm::mat4(1.0f), js.position_ * glm::normalize(axis));
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
        traverse(urdf_model_->getRoot(), glm::mat4(1.0f));
    }

    void RobotLoader::updateJointTransform(const std::string& joint_name, float new_position) {
        for (auto& js : joint_states_) {
            if (js.name_ == joint_name) {
                js.position_ = new_position;
                break;
            }
        }
        updateTransforms();
    }

}  // namespace omnilink::gui