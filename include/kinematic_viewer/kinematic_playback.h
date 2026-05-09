#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"

namespace omnilink::teleop_viewer {
class RobotScene;
}

namespace kinematic_viewer {

void ApplyPlaybackStep(DebugPlaybackState* playbackState, omnilink::teleop_viewer::RobotScene* scene, double dtSec);

}  // namespace kinematic_viewer
