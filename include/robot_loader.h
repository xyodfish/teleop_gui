#pragma once

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

#include "robot_mesh_model.h"

namespace omnilink::gui {

    struct JointState {
        std::string name_;
        float position_  = 0.0f;  // radians
        float min_angle_ = -3.14f;
        float max_angle_ = 3.14f;
    };

    struct LinkVisual {
        std::string mesh_file_;
        glm::vec3 origin_xyz_ = glm::vec3(0);
        glm::vec3 origin_rpy_ = glm::vec3(0);
        glm::vec3 scale_      = glm::vec3(1);

        // <<< MOD: 父 link 名称 & visual 相对于 link 的本地变换（不包含 link 全局变换）
        std::string parent_link_name_;
        glm::mat4 local_transform_ = glm::mat4(1.0f);

        Model model_;
        bool loaded_ = false;
    };

    class RobotLoader {
       public:
        std::map<std::string, LinkVisual> visuals_;
        std::map<std::string, glm::mat4> transforms_;  // link_name -> global
        std::vector<JointState> joint_states_;
        std::string package_path_;                  // URDF 文件所在目录（你的原始实现）
        urdf::ModelInterfaceSharedPtr urdf_model_;  // <<< MOD: 保存解析后的 model 指针
        std::string urdf_file_path_;                // 保存路径（若需要）

        std::string resolvePath(const std::string& path);
        void loadURDF(const std::string& urdf_path);
        void initJointStates(urdf::ModelInterfaceSharedPtr model);
        void draw(GLuint shader, const glm::mat4& view, const glm::mat4& proj);

        void updateTransforms();

        void updateJointTransform(const std::string& joint_name, float new_position);

        static RobotLoader& Instance() {
            static RobotLoader instance;
            return instance;
        }

        // 禁用拷贝和赋值
        RobotLoader(const RobotLoader&) = delete;
        RobotLoader& operator=(const RobotLoader&) = delete;
        RobotLoader()                              = default;
    };
}  // namespace omnilink::gui
