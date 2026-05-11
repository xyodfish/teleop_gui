#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <vector>

namespace kinematic_viewer {

void RenderScenePanel(ViewerState* uiState);
void RenderJointPanel(ViewerState* uiState, omnilink::teleop_viewer::RobotScene* scene,
                      const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints);
void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer,
                         omnilink::teleop_viewer::RobotScene* scene,
                         const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints);
void RenderSafetyPanel(CollisionMonitorState* collisionState, const CollisionMonitorResult& collisionResult);
void RenderObstaclePanel(ViewerState* uiState);
void RenderTfPanel(ViewerState* uiState, const std::vector<omnilink::teleop_viewer::RobotScene::LinkTfInfo>& tfs);

}  // namespace kinematic_viewer
