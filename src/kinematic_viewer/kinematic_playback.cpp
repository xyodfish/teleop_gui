#include "kinematic_viewer/kinematic_playback.h"

#include "teleop_viewer/scene.h"

#include <algorithm>

namespace kinematic_viewer {

void ApplyPlaybackStep(DebugPlaybackState* playbackState, omnilink::teleop_viewer::RobotScene* scene, double dtSec) {
    if (playbackState == nullptr || scene == nullptr) {
        return;
    }
    if (!playbackState->playing || playbackState->keyframes.size() < 2) {
        return;
    }

    const float total = static_cast<float>(playbackState->keyframes.back().t);
    playbackState->play_time += static_cast<float>(dtSec) * playbackState->play_speed;
    if (playbackState->loop && total > 1e-4f) {
        while (playbackState->play_time > total) {
            playbackState->play_time -= total;
        }
    } else if (playbackState->play_time > total) {
        playbackState->play_time = total;
        playbackState->playing   = false;
    }

    size_t hi = 1;
    while (hi < playbackState->keyframes.size() && static_cast<float>(playbackState->keyframes[hi].t) < playbackState->play_time) {
        ++hi;
    }
    size_t lo      = (hi == 0) ? 0 : (hi - 1);
    hi             = std::min(hi, playbackState->keyframes.size() - 1);
    const auto& k0 = playbackState->keyframes[lo];
    const auto& k1 = playbackState->keyframes[hi];
    float t0       = static_cast<float>(k0.t);
    float t1       = static_cast<float>(k1.t);
    float alpha    = (t1 > t0 + 1e-6f) ? ((playbackState->play_time - t0) / (t1 - t0)) : 0.0f;
    alpha          = std::clamp(alpha, 0.0f, 1.0f);

    auto jointsNow = scene->getJointInfos();
    for (const auto& j : jointsNow) {
        auto it0 = k0.joints.find(j.name);
        auto it1 = k1.joints.find(j.name);
        if (it0 == k0.joints.end() || it1 == k1.joints.end()) {
            continue;
        }
        float v = it0->second * (1.0f - alpha) + it1->second * alpha;
        scene->setJointPositionByName(j.name, v);
    }
}

}  // namespace kinematic_viewer
