#include "kinematic_viewer/kinematic_sidebar_panels.h"

#include "imgui.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <string>

namespace kinematic_viewer {

void RenderScenePanel(ViewerState* uiState) {
    if (uiState == nullptr) {
        return;
    }
    ImGui::Checkbox("显示关节轴", &uiState->show_axes);
    ImGui::Checkbox("仅旋转关节轴", &uiState->show_revolute_only);
    ImGui::Checkbox("显示非旋转关节", &uiState->show_non_revolute);
    ImGui::Checkbox("显示世界坐标轴", &uiState->show_world_axes);
    ImGui::Checkbox("固定底座模式", &uiState->lock_base);
    ImGui::SliderFloat("关节轴长度", &uiState->axis_length, 0.03f, 0.5f, "%.3f");
    ImGui::SliderFloat("线宽", &uiState->axis_line_width, 1.0f, 6.0f, "%.1f");
    ImGui::SliderFloat("世界轴长度", &uiState->world_axis_length, 0.1f, 1.5f, "%.2f");
    ImGui::SliderFloat("地面网格尺寸", &uiState->grid_size, 1.0f, 20.0f, "%.1f");
    ImGui::SliderInt("地面网格密度", &uiState->grid_count, 10, 120);
    ImGui::Separator();
}

void RenderJointPanel(ViewerState* uiState, omnilink::teleop_viewer::RobotScene* scene,
                      const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
    if (uiState == nullptr || scene == nullptr) {
        return;
    }

    int revoluteCount   = 0;
    int clampedCount    = 0;
    float minMarginDeg  = 1e9f;
    std::string minName = "";
    for (const auto& j : joints) {
        if (j.revolute) {
            ++revoluteCount;
        }
        if (j.position < j.min_angle - 1e-5f || j.position > j.max_angle + 1e-5f) {
            ++clampedCount;
        }
        if (j.revolute) {
            float d0 = std::fabs(j.position - j.min_angle);
            float d1 = std::fabs(j.max_angle - j.position);
            float m  = glm::degrees(std::min(d0, d1));
            if (m < minMarginDeg) {
                minMarginDeg = m;
                minName      = j.name;
            }
        }
    }

    ImGui::InputText("关节过滤", uiState->joint_filter, sizeof(uiState->joint_filter));
    std::string filter = uiState->joint_filter;
    std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    ImGui::Text("关节总数: %d  旋转关节: %d  越界关节: %d", static_cast<int>(joints.size()), revoluteCount, clampedCount);
    if (!minName.empty()) {
        ImVec4 c = (minMarginDeg < 3.0f) ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
                                         : ((minMarginDeg < 8.0f) ? ImVec4(1.0f, 0.75f, 0.25f, 1.0f)
                                                                  : ImVec4(0.6f, 0.9f, 0.6f, 1.0f));
        ImGui::TextColored(c, "最小限位裕量: %.2f deg (%s)", minMarginDeg, minName.c_str());
    }

    if (ImGui::Button("旋转关节全部归零")) {
        for (const auto& j : joints) {
            if (j.revolute) {
                scene->setJointPositionByName(j.name, 0.0f);
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("一键夹紧到限位内")) {
        for (const auto& j : joints) {
            if (!j.revolute) {
                continue;
            }
            float v = std::clamp(j.position, j.min_angle, j.max_angle);
            scene->setJointPositionByName(j.name, v);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("保存当前姿态")) {
        uiState->pose_snapshot.clear();
        for (const auto& j : joints) {
            uiState->pose_snapshot[j.name] = j.position;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("恢复保存姿态") && !uiState->pose_snapshot.empty()) {
        for (const auto& [name, value] : uiState->pose_snapshot) {
            scene->setJointPositionByName(name, value);
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("关节调试");
    if (ImGui::BeginTable("joint_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                          ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("关节");
        ImGui::TableSetupColumn("角度滑条");
        ImGui::TableSetupColumn("限位");
        ImGui::TableSetupColumn("数值输入(度)");
        ImGui::TableHeadersRow();

        for (const auto& j : joints) {
            if (!uiState->show_non_revolute && !j.revolute) {
                continue;
            }
            std::string nameLower = j.name;
            std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (!filter.empty() && nameLower.find(filter) == std::string::npos) {
                continue;
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(j.name.c_str());
            ImGui::TableSetColumnIndex(1);
            float value = j.position;
            if (j.revolute) {
                ImGui::PushItemWidth(-1);
                std::string sliderId = "##slider_" + j.name;
                if (ImGui::SliderAngle(sliderId.c_str(), &value, glm::degrees(j.min_angle), glm::degrees(j.max_angle))) {
                    scene->setJointPositionByName(j.name, value);
                }
                ImGui::PopItemWidth();
            } else {
                ImGui::TextDisabled("不适用");
            }

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.2f / %.2f deg", glm::degrees(j.min_angle), glm::degrees(j.max_angle));
            ImGui::TableSetColumnIndex(3);
            float degreeVal     = glm::degrees(j.position);
            std::string inputId = "##deg_" + j.name;
            if (ImGui::InputFloat(inputId.c_str(), &degreeVal, 0.1f, 1.0f, "%.2f")) {
                float rad = glm::radians(degreeVal);
                scene->setJointPositionByName(j.name, rad);
            }
        }
        ImGui::EndTable();
    }
}

void RenderPlaybackPanel(DebugPlaybackState* playbackState, const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
    if (playbackState == nullptr) {
        return;
    }
    ImGui::Separator();
    ImGui::TextUnformatted("姿态关键帧回放");
    if (ImGui::Button("记录关键帧")) {
        PoseKeyframe kf;
        kf.t = playbackState->keyframes.empty() ? 0.0 : (playbackState->keyframes.back().t + 1.0);
        for (const auto& j : joints) {
            if (j.revolute) {
                kf.joints[j.name] = j.position;
            }
        }
        playbackState->keyframes.push_back(std::move(kf));
        playbackState->play_time = static_cast<float>(playbackState->keyframes.back().t);
    }
    ImGui::SameLine();
    if (ImGui::Button(playbackState->playing ? "暂停回放" : "开始回放")) {
        if (playbackState->keyframes.size() >= 2) {
            playbackState->playing = !playbackState->playing;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("清空关键帧")) {
        playbackState->keyframes.clear();
        playbackState->playing   = false;
        playbackState->play_time = 0.0f;
    }
    ImGui::Checkbox("循环回放", &playbackState->loop);
    ImGui::SliderFloat("回放倍速", &playbackState->play_speed, 0.1f, 3.0f, "%.2fx");
    if (!playbackState->keyframes.empty()) {
        float total = static_cast<float>(playbackState->keyframes.back().t);
        ImGui::SliderFloat("回放时间", &playbackState->play_time, 0.0f, std::max(0.0f, total), "%.2f s");
        ImGui::Text("关键帧数: %d", static_cast<int>(playbackState->keyframes.size()));
    } else {
        ImGui::TextDisabled("暂无关键帧，先点击“记录关键帧”。");
    }
}

void RenderTfPanel(ViewerState* uiState, const std::vector<omnilink::teleop_viewer::RobotScene::LinkTfInfo>& tfs) {
    if (uiState == nullptr) {
        return;
    }
    ImGui::Separator();
    ImGui::TextUnformatted("TF 视图");
    ImGui::InputText("TF过滤", uiState->tf_filter, sizeof(uiState->tf_filter));
    std::string tfFilter = uiState->tf_filter;
    std::transform(tfFilter.begin(), tfFilter.end(), tfFilter.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    if (ImGui::BeginTable("tf_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                          ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupColumn("Link");
        ImGui::TableSetupColumn("父Link");
        ImGui::TableSetupColumn("位置 xyz(m)");
        ImGui::TableSetupColumn("姿态 rpy(deg)");
        ImGui::TableHeadersRow();
        for (const auto& tf : tfs) {
            std::string key      = tf.name + " " + tf.parent_name;
            std::string keyLower = key;
            std::transform(keyLower.begin(), keyLower.end(), keyLower.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (!tfFilter.empty() && keyLower.find(tfFilter) == std::string::npos) {
                continue;
            }
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(tf.name.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(tf.parent_name.empty() ? "-" : tf.parent_name.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f, %.3f, %.3f", tf.world_position.x, tf.world_position.y, tf.world_position.z);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.1f, %.1f, %.1f", glm::degrees(tf.world_rpy.x), glm::degrees(tf.world_rpy.y), glm::degrees(tf.world_rpy.z));
        }
        ImGui::EndTable();
    }
}

}  // namespace kinematic_viewer
