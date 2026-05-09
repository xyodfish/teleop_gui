#include "kinematic_viewer/kinematic_sidebar_panels.h"

#include "imgui.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace kinematic_viewer {
namespace kinematic_sidebar_panels_internal {

std::string NormalizePath(const std::string& path) {
    std::error_code ec;
    std::filesystem::path p(path);
    auto normalized = std::filesystem::weakly_canonical(p, ec);
    if (!ec) {
        return normalized.string();
    }
    return p.lexically_normal().string();
}

bool IsTrajectoryFileExt(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return ext == ".yaml" || ext == ".yml" || ext == ".csv";
}

bool ValidateTrajectoryJointNames(const DebugPlaybackState& playbackState,
                                  const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints,
                                  std::string* errorMessage) {
    std::unordered_set<std::string> sceneJointNames;
    for (const auto& joint : joints) {
        sceneJointNames.insert(joint.name);
    }

    std::unordered_set<std::string> trajectoryJointNames;
    for (const auto& keyframe : playbackState.keyframes) {
        for (const auto& [jointName, _] : keyframe.joints) {
            trajectoryJointNames.insert(jointName);
        }
    }

    if (trajectoryJointNames.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "轨迹文件中未找到任何关节名";
        }
        return false;
    }

    std::vector<std::string> unknown;
    int matchedCount = 0;
    for (const auto& jointName : trajectoryJointNames) {
        if (sceneJointNames.find(jointName) == sceneJointNames.end()) {
            unknown.push_back(jointName);
        } else {
            ++matchedCount;
        }
    }

    if (!unknown.empty()) {
        std::sort(unknown.begin(), unknown.end());
        std::stringstream ss;
        ss << "轨迹关节名与当前机器人不匹配，未知关节 " << unknown.size() << " 个: ";
        const size_t showCount = std::min<size_t>(unknown.size(), 8);
        for (size_t i = 0; i < showCount; ++i) {
            if (i > 0) {
                ss << ", ";
            }
            ss << unknown[i];
        }
        if (unknown.size() > showCount) {
            ss << " ...";
        }
        if (errorMessage != nullptr) {
            *errorMessage = ss.str();
        }
        return false;
    }

    if (matchedCount <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = "轨迹关节名与当前机器人无任何匹配";
        }
        return false;
    }

    return true;
}

void RenderTrajectoryFileBrowser(DebugPlaybackState* playbackState) {
    if (playbackState == nullptr) {
        return;
    }

    if (ImGui::Button("浏览本地文件")) {
        const std::string defaultDir = NormalizePath(std::filesystem::current_path().string());
        std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", defaultDir.c_str());
        ImGui::OpenPopup("trajectory_file_browser_popup");
    }

    if (!ImGui::BeginPopupModal("trajectory_file_browser_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        return;
    }

    ImGui::InputText("目录", playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir));
    ImGui::SameLine();
    if (ImGui::Button("进入目录")) {
        const std::string normalized = NormalizePath(playbackState->trajectory_browser_dir);
        std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", normalized.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button("上一级")) {
        std::filesystem::path current = std::filesystem::path(NormalizePath(playbackState->trajectory_browser_dir));
        std::filesystem::path parent  = current.parent_path();
        if (parent.empty()) {
            parent = std::filesystem::path("/");
        }
        std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", parent.string().c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button("根目录/")) {
        std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", "/");
    }
    ImGui::SameLine();
    if (ImGui::Button("HOME")) {
        const char* home = std::getenv("HOME");
        if (home != nullptr && home[0] != '\0') {
            std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", home);
        }
    }
    ImGui::TextDisabled("支持输入任意绝对路径，例如 /home/user/data");

    std::error_code ec;
    const std::filesystem::path browsePath(playbackState->trajectory_browser_dir);
    if (!std::filesystem::exists(browsePath, ec) || !std::filesystem::is_directory(browsePath, ec)) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
    } else {
        std::vector<std::filesystem::path> dirs;
        std::vector<std::filesystem::path> files;
        for (auto it = std::filesystem::directory_iterator(browsePath, ec); !ec && it != std::filesystem::directory_iterator(); ++it) {
            if (it->is_directory(ec)) {
                dirs.push_back(it->path());
            } else if (it->is_regular_file(ec) && IsTrajectoryFileExt(it->path())) {
                files.push_back(it->path());
            }
        }
        std::sort(dirs.begin(), dirs.end());
        std::sort(files.begin(), files.end());

        if (ImGui::BeginChild("trajectory_file_browser_list", ImVec2(580, 280), true)) {
            for (const auto& d : dirs) {
                std::string label = "[DIR] " + d.filename().string();
                if (ImGui::Selectable(label.c_str(), false)) {
                    std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", d.string().c_str());
                }
            }
            for (const auto& f : files) {
                std::string label = f.filename().string();
                if (ImGui::Selectable(label.c_str(), false)) {
                    const std::string selectedPath = f.string();
                    std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s", selectedPath.c_str());
                    ImGui::CloseCurrentPopup();
                }
            }
            ImGui::EndChild();
        }
    }

    if (ImGui::Button("关闭")) {
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

}  // namespace kinematic_sidebar_panels_internal

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

void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer,
                         omnilink::teleop_viewer::RobotScene* scene,
                         const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
    if (playbackState == nullptr || playbackPlayer == nullptr || scene == nullptr) {
        return;
    }

    constexpr const char* kTrajectoryAlertPopupId = "轨迹告警##trajectory_incompatible_alert_popup";
    if (playbackState->trajectory_alert_popup_pending) {
        ImGui::OpenPopup(kTrajectoryAlertPopupId);
        playbackState->trajectory_alert_popup_pending = false;
    }
    ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f), ImGuiCond_Appearing);
    if (ImGui::BeginPopupModal(kTrajectoryAlertPopupId, nullptr, ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f), "轨迹文件不适用");
        ImGui::Separator();
        ImGui::TextWrapped("%s", playbackState->trajectory_alert_message.empty() ? "该轨迹无法用于当前机器人。" : playbackState->trajectory_alert_message.c_str());
        if (!playbackState->trajectory_alert_detail.empty() && ImGui::CollapsingHeader("查看详情")) {
            ImGui::TextWrapped("%s", playbackState->trajectory_alert_detail.c_str());
        }
        ImGui::Spacing();
        if (ImGui::Button("知道了", ImVec2(120.0f, 0.0f))) {
            playbackState->trajectory_alert_detail.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("轨迹关键帧回放");
    ImGui::DragFloat("关键帧间隔(s)", &playbackState->keyframe_interval_sec, 0.02f, 0.02f, 5.0f, "%.2f");
    ImGui::InputText("轨迹文件", playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path));
    ImGui::SameLine();
    kinematic_sidebar_panels_internal::RenderTrajectoryFileBrowser(playbackState);
    if (ImGui::Button("加载轨迹文件")) {
        const DebugPlaybackState previousState = *playbackState;
        std::string ioError;
        if (LoadTrajectoryFromFile(playbackState->trajectory_file_path, playbackState, &ioError)) {
            std::string checkError;
            if (!kinematic_sidebar_panels_internal::ValidateTrajectoryJointNames(*playbackState, joints, &checkError)) {
                *playbackState = previousState;
                playbackState->trajectory_io_status = "加载失败: " + checkError;
                playbackState->trajectory_alert_message = "该轨迹与当前机器人关节定义不匹配。";
                playbackState->trajectory_alert_detail = checkError;
                playbackState->trajectory_alert_popup_pending = true;
            } else {
                playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                playbackState->trajectory_io_status = "加载成功";
            }
        } else {
            playbackState->trajectory_io_status = "加载失败: " + ioError;
            playbackState->trajectory_alert_message = "轨迹文件加载失败，请检查路径或文件格式。";
            playbackState->trajectory_alert_detail = ioError;
            playbackState->trajectory_alert_popup_pending = true;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("保存当前轨迹")) {
        std::string ioError;
        if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
            playbackState->trajectory_io_status = "保存成功";
        } else {
            playbackState->trajectory_io_status = "保存失败: " + ioError;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("生成Demo轨迹")) {
        BuildDemoTrajectoryFromCurrentPose(playbackState, joints);
        std::string ioError;
        if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
            playbackState->trajectory_io_status = "Demo轨迹已生成并保存";
        } else {
            playbackState->trajectory_io_status = "Demo生成成功但保存失败: " + ioError;
        }
        playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
    }
    if (!playbackState->trajectory_io_status.empty()) {
        ImGui::TextDisabled("%s", playbackState->trajectory_io_status.c_str());
    }
    ImGui::Separator();
    if (ImGui::Button("记录关键帧")) {
        playbackPlayer->RecordKeyframe(playbackState, joints);
    }
    ImGui::SameLine();
    const bool playing = playbackState->mode == DebugPlaybackState::Mode::Playing;
    if (ImGui::Button(playing ? "暂停回放" : "开始回放")) {
        playbackPlayer->TogglePlayPause(playbackState);
    }
    ImGui::SameLine();
    if (ImGui::Button("停止")) {
        playbackPlayer->Stop(playbackState);
    }
    ImGui::SameLine();
    if (ImGui::Button("清空")) {
        playbackPlayer->Clear(playbackState);
    }
    ImGui::Checkbox("循环回放", &playbackState->loop);
    ImGui::SliderFloat("回放倍速", &playbackState->play_speed, 0.1f, 3.0f, "%.2fx");

    if (!playbackState->keyframes.empty() && playbackState->selected_keyframe_index >= 0 &&
        playbackState->selected_keyframe_index < static_cast<int>(playbackState->keyframes.size())) {
        ImGui::SameLine();
        if (ImGui::Button("删除选中关键帧")) {
            playbackPlayer->RemoveSelectedKeyframe(playbackState);
        }
    }

    if (!playbackState->keyframes.empty()) {
        float total = TrajectoryPlayer::TotalDuration(*playbackState);
        if (ImGui::SliderFloat("回放时间", &playbackState->play_time, 0.0f, std::max(0.0f, total), "%.2f s")) {
            playbackState->timeline_edited_this_ui = true;
        }
        if (playbackState->timeline_edited_this_ui) {
            playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
            playbackState->timeline_edited_this_ui = false;
        }

        const char* modeLabel = "Stopped";
        if (playbackState->mode == DebugPlaybackState::Mode::Playing) {
            modeLabel = "Playing";
        } else if (playbackState->mode == DebugPlaybackState::Mode::Paused) {
            modeLabel = "Paused";
        }
        ImGui::Text("状态: %s  总时长: %.2fs  当前段: %d", modeLabel, total, playbackState->current_segment_index);
        ImGui::Text("关键帧数: %d", static_cast<int>(playbackState->keyframes.size()));

        if (ImGui::BeginTable("keyframe_table", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              ImVec2(0.0f, 180.0f))) {
            ImGui::TableSetupColumn("索引");
            ImGui::TableSetupColumn("时间(s)");
            ImGui::TableSetupColumn("关节数");
            ImGui::TableHeadersRow();
            for (int i = 0; i < static_cast<int>(playbackState->keyframes.size()); ++i) {
                const auto& keyframe = playbackState->keyframes[static_cast<size_t>(i)];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                char selectLabel[32];
                snprintf(selectLabel, sizeof(selectLabel), "KF %d", i);
                if (ImGui::Selectable(selectLabel, playbackState->selected_keyframe_index == i, ImGuiSelectableFlags_SpanAllColumns)) {
                    playbackState->selected_keyframe_index = i;
                    playbackState->play_time = static_cast<float>(keyframe.t);
                    playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", keyframe.t);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%d", static_cast<int>(keyframe.joints.size()));
            }
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("暂无关键帧，先点击“记录关键帧”。");
    }
}

void RenderSafetyPanel(CollisionMonitorState* collisionState, const CollisionMonitorResult& collisionResult) {
    if (collisionState == nullptr) {
        return;
    }
    ImGui::Separator();
    ImGui::TextUnformatted("碰撞预警与距离监控");
    ImGui::Checkbox("启用碰撞监控", &collisionState->enable);
    ImGui::Checkbox("忽略同一Link", &collisionState->ignore_same_link);
    ImGui::Checkbox("忽略父子Link", &collisionState->ignore_parent_child);
    ImGui::Checkbox("显示最近对连线", &collisionState->show_closest_pair_line);
    ImGui::DragFloat("Danger阈值(m)", &collisionState->danger_distance_m, 0.002f, -0.20f, 0.30f, "%.3f");
    ImGui::DragFloat("Warning阈值(m)", &collisionState->warning_distance_m, 0.002f, -0.20f, 0.50f, "%.3f");
    if (collisionState->warning_distance_m < collisionState->danger_distance_m) {
        collisionState->warning_distance_m = collisionState->danger_distance_m;
    }

    ImGui::Text("评估Pair数: %d", collisionState->evaluated_pair_count);
    ImGui::Text("Warning对数: %d  Danger对数: %d", collisionResult.warning_pair_count, collisionResult.danger_pair_count);
    if (!collisionState->has_valid_distance) {
        ImGui::TextDisabled("暂无可用距离数据（可能proxy不足或全部被过滤）");
        return;
    }

    ImVec4 color(0.60f, 0.95f, 0.60f, 1.0f);
    if (collisionState->nearest_surface_distance_m <= collisionState->danger_distance_m) {
        color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
    } else if (collisionState->nearest_surface_distance_m <= collisionState->warning_distance_m) {
        color = ImVec4(1.0f, 0.80f, 0.30f, 1.0f);
    }

    ImGui::Text("最近Link对: %s <-> %s", collisionState->nearest_link_a.c_str(), collisionState->nearest_link_b.c_str());
    ImGui::TextColored(color, "最近表面距离: %.3f m", collisionState->nearest_surface_distance_m);
    ImGui::Text("中心距离: %.3f m", collisionState->nearest_center_distance_m);
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
