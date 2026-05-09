#include "kinematic_viewer/kinematic_playback.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace kinematic_viewer {
namespace kinematic_playback_internal {

DebugPlaybackState::Mode NextPausedOrPlaying(DebugPlaybackState::Mode mode) {
    if (mode == DebugPlaybackState::Mode::Playing) {
        return DebugPlaybackState::Mode::Paused;
    }
    return DebugPlaybackState::Mode::Playing;
}

float WrapPhase(float phase) {
    const float twoPi = 6.283185307f;
    while (phase > twoPi) {
        phase -= twoPi;
    }
    while (phase < 0.0f) {
        phase += twoPi;
    }
    return phase;
}

bool ParseLegacyKeyframesNode(const YAML::Node& keyframesNode, std::vector<PoseKeyframe>* outKeyframes) {
    if (!keyframesNode || !keyframesNode.IsSequence() || outKeyframes == nullptr) {
        return false;
    }
    std::vector<PoseKeyframe> loaded;
    loaded.reserve(keyframesNode.size());
    for (const auto& item : keyframesNode) {
        if (!item || !item.IsMap()) {
            continue;
        }
        PoseKeyframe keyframe;
        keyframe.t = item["t"] ? item["t"].as<double>() : 0.0;
        YAML::Node joints = item["joints"];
        if (joints && joints.IsMap()) {
            for (auto it = joints.begin(); it != joints.end(); ++it) {
                if (!it->first.IsScalar() || !it->second.IsScalar()) {
                    continue;
                }
                keyframe.joints[it->first.as<std::string>()] = it->second.as<float>();
            }
        }
        loaded.push_back(std::move(keyframe));
    }
    std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
    *outKeyframes = std::move(loaded);
    return true;
}

bool ParseCompactSamplesNode(const YAML::Node& trajectoryNode, std::vector<PoseKeyframe>* outKeyframes, std::string* errorMessage) {
    if (!trajectoryNode || !trajectoryNode.IsMap() || outKeyframes == nullptr) {
        return false;
    }
    YAML::Node jointsNode = trajectoryNode["joints"];
    YAML::Node samplesNode = trajectoryNode["samples"];
    if (!jointsNode || !jointsNode.IsSequence() || !samplesNode || !samplesNode.IsSequence()) {
        return false;
    }

    std::vector<std::string> jointNames;
    jointNames.reserve(jointsNode.size());
    for (const auto& joint : jointsNode) {
        if (!joint.IsScalar()) {
            continue;
        }
        jointNames.push_back(joint.as<std::string>());
    }
    if (jointNames.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "compact samples: joints is empty";
        }
        return false;
    }

    std::vector<PoseKeyframe> loaded;
    loaded.reserve(samplesNode.size());
    for (const auto& row : samplesNode) {
        if (!row || !row.IsSequence() || row.size() < (jointNames.size() + 1)) {
            continue;
        }
        PoseKeyframe keyframe;
        keyframe.t = row[0].as<double>();
        for (size_t i = 0; i < jointNames.size(); ++i) {
            keyframe.joints[jointNames[i]] = row[i + 1].as<float>();
        }
        loaded.push_back(std::move(keyframe));
    }
    std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
    *outKeyframes = std::move(loaded);
    return true;
}

bool ParseCompactDtValuesNode(const YAML::Node& trajectoryNode, std::vector<PoseKeyframe>* outKeyframes, std::string* errorMessage) {
    if (!trajectoryNode || !trajectoryNode.IsMap() || outKeyframes == nullptr) {
        return false;
    }
    YAML::Node jointsNode = trajectoryNode["joints"];
    YAML::Node valuesNode = trajectoryNode["values"];
    YAML::Node dtNode = trajectoryNode["dt"];
    if (!jointsNode || !jointsNode.IsSequence() || !valuesNode || !valuesNode.IsSequence() || !dtNode || !dtNode.IsScalar()) {
        return false;
    }

    const double dt = dtNode.as<double>();
    const double t0 = trajectoryNode["t0"] ? trajectoryNode["t0"].as<double>() : 0.0;
    if (dt <= 0.0) {
        if (errorMessage != nullptr) {
            *errorMessage = "compact dt-values: dt must be > 0";
        }
        return false;
    }

    std::vector<std::string> jointNames;
    jointNames.reserve(jointsNode.size());
    for (const auto& joint : jointsNode) {
        if (!joint.IsScalar()) {
            continue;
        }
        jointNames.push_back(joint.as<std::string>());
    }
    if (jointNames.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "compact dt-values: joints is empty";
        }
        return false;
    }

    std::vector<PoseKeyframe> loaded;
    loaded.reserve(valuesNode.size());
    for (size_t rowIndex = 0; rowIndex < valuesNode.size(); ++rowIndex) {
        const auto& row = valuesNode[rowIndex];
        if (!row || !row.IsSequence() || row.size() < jointNames.size()) {
            continue;
        }
        PoseKeyframe keyframe;
        keyframe.t = t0 + dt * static_cast<double>(rowIndex);
        for (size_t i = 0; i < jointNames.size(); ++i) {
            keyframe.joints[jointNames[i]] = row[i].as<float>();
        }
        loaded.push_back(std::move(keyframe));
    }

    *outKeyframes = std::move(loaded);
    return true;
}

std::vector<std::string> CollectJointNames(const DebugPlaybackState& playbackState) {
    std::vector<std::string> names;
    for (const auto& keyframe : playbackState.keyframes) {
        for (const auto& [jointName, _] : keyframe.joints) {
            if (std::find(names.begin(), names.end(), jointName) == names.end()) {
                names.push_back(jointName);
            }
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

bool IsUniformTimeStep(const DebugPlaybackState& playbackState, double* outDt, double* outT0) {
    if (playbackState.keyframes.size() < 2 || outDt == nullptr || outT0 == nullptr) {
        return false;
    }
    const double dt = playbackState.keyframes[1].t - playbackState.keyframes[0].t;
    if (dt <= 0.0) {
        return false;
    }
    constexpr double kTol = 1e-6;
    for (size_t i = 2; i < playbackState.keyframes.size(); ++i) {
        const double step = playbackState.keyframes[i].t - playbackState.keyframes[i - 1].t;
        if (std::fabs(step - dt) > kTol) {
            return false;
        }
    }
    *outDt = dt;
    *outT0 = playbackState.keyframes[0].t;
    return true;
}

std::string Trim(const std::string& input) {
    size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
        ++begin;
    }
    size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }
    return input.substr(begin, end - begin);
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        out.push_back(Trim(cell));
    }
    if (!line.empty() && line.back() == ',') {
        out.emplace_back("");
    }
    return out;
}

std::string LowerString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return s;
}

bool EndsWithCaseInsensitive(const std::string& value, const std::string& suffix) {
    if (value.size() < suffix.size()) {
        return false;
    }
    const std::string left  = LowerString(value.substr(value.size() - suffix.size()));
    const std::string right = LowerString(suffix);
    return left == right;
}

std::string LowerFileExtension(const std::string& path) {
    std::filesystem::path p(path);
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return ext;
}

}  // namespace kinematic_playback_internal

void LinearTrajectoryInterpolator::SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec,
                                                  int* currentSegmentIndex,
                                                  omnilink::teleop_viewer::RobotScene* scene) const {
    if (scene == nullptr || playbackState.keyframes.empty()) {
        return;
    }

    if (playbackState.keyframes.size() == 1) {
        const auto& only = playbackState.keyframes.front();
        for (const auto& [jointName, value] : only.joints) {
            scene->setJointPositionByName(jointName, value);
        }
        if (currentSegmentIndex != nullptr) {
            *currentSegmentIndex = 0;
        }
        return;
    }

    size_t hi = 1;
    while (hi < playbackState.keyframes.size() && static_cast<float>(playbackState.keyframes[hi].t) < sampleTimeSec) {
        ++hi;
    }
    size_t lo = (hi == 0) ? 0 : (hi - 1);
    hi        = std::min(hi, playbackState.keyframes.size() - 1);

    const auto& k0 = playbackState.keyframes[lo];
    const auto& k1 = playbackState.keyframes[hi];
    const float t0 = static_cast<float>(k0.t);
    const float t1 = static_cast<float>(k1.t);
    float alpha    = (t1 > t0 + 1e-6f) ? ((sampleTimeSec - t0) / (t1 - t0)) : 0.0f;
    alpha          = std::clamp(alpha, 0.0f, 1.0f);

    for (const auto& [jointName, value0] : k0.joints) {
        auto it1 = k1.joints.find(jointName);
        if (it1 == k1.joints.end()) {
            continue;
        }
        float value = value0 * (1.0f - alpha) + it1->second * alpha;
        scene->setJointPositionByName(jointName, value);
    }

    if (currentSegmentIndex != nullptr) {
        *currentSegmentIndex = static_cast<int>(lo);
    }
}

TrajectoryPlayer::TrajectoryPlayer() : interpolator_(std::make_unique<LinearTrajectoryInterpolator>()) {}

void TrajectoryPlayer::SetInterpolator(std::unique_ptr<TrajectoryInterpolator> interpolator) {
    if (interpolator) {
        interpolator_ = std::move(interpolator);
    }
}

void TrajectoryPlayer::RecordKeyframe(DebugPlaybackState* playbackState,
                                      const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) const {
    if (playbackState == nullptr) {
        return;
    }

    PoseKeyframe keyframe;
    if (!playbackState->keyframes.empty()) {
        keyframe.t = playbackState->keyframes.back().t + std::max(0.02f, playbackState->keyframe_interval_sec);
    }
    for (const auto& joint : joints) {
        if (!joint.revolute) {
            continue;
        }
        keyframe.joints[joint.name] = joint.position;
    }
    playbackState->keyframes.push_back(std::move(keyframe));
    playbackState->selected_keyframe_index = static_cast<int>(playbackState->keyframes.size()) - 1;
    playbackState->play_time               = static_cast<float>(playbackState->keyframes.back().t);
}

void TrajectoryPlayer::RemoveSelectedKeyframe(DebugPlaybackState* playbackState) const {
    if (playbackState == nullptr || playbackState->keyframes.empty()) {
        return;
    }
    const int index = playbackState->selected_keyframe_index;
    if (index < 0 || index >= static_cast<int>(playbackState->keyframes.size())) {
        return;
    }
    playbackState->keyframes.erase(playbackState->keyframes.begin() + index);
    if (playbackState->keyframes.empty()) {
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->selected_keyframe_index = -1;
        playbackState->play_time               = 0.0f;
        playbackState->current_segment_index   = -1;
        return;
    }
    playbackState->selected_keyframe_index = std::clamp(index, 0, static_cast<int>(playbackState->keyframes.size()) - 1);
    playbackState->play_time = std::min(playbackState->play_time, TotalDuration(*playbackState));
}

void TrajectoryPlayer::Clear(DebugPlaybackState* playbackState) const {
    if (playbackState == nullptr) {
        return;
    }
    playbackState->keyframes.clear();
    playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
    playbackState->play_time               = 0.0f;
    playbackState->selected_keyframe_index = -1;
    playbackState->current_segment_index   = -1;
}

void TrajectoryPlayer::TogglePlayPause(DebugPlaybackState* playbackState) const {
    if (playbackState == nullptr || !HasPlayableTrajectory(*playbackState)) {
        return;
    }
    if (playbackState->mode == DebugPlaybackState::Mode::Stopped) {
        playbackState->play_time = 0.0f;
        playbackState->mode      = DebugPlaybackState::Mode::Playing;
        return;
    }
    playbackState->mode = kinematic_playback_internal::NextPausedOrPlaying(playbackState->mode);
}

void TrajectoryPlayer::Stop(DebugPlaybackState* playbackState) const {
    if (playbackState == nullptr) {
        return;
    }
    playbackState->mode      = DebugPlaybackState::Mode::Stopped;
    playbackState->play_time = 0.0f;
}

void TrajectoryPlayer::AdvanceAndApply(DebugPlaybackState* playbackState, omnilink::teleop_viewer::RobotScene* scene,
                                       double dtSec) const {
    if (playbackState == nullptr || scene == nullptr || !interpolator_) {
        return;
    }
    if (playbackState->mode != DebugPlaybackState::Mode::Playing || !HasPlayableTrajectory(*playbackState)) {
        return;
    }

    const float totalDuration = TotalDuration(*playbackState);
    if (totalDuration <= 1e-6f) {
        playbackState->mode = DebugPlaybackState::Mode::Stopped;
        return;
    }

    playbackState->play_time += static_cast<float>(dtSec) * playbackState->play_speed;
    if (playbackState->loop) {
        while (playbackState->play_time > totalDuration) {
            playbackState->play_time -= totalDuration;
        }
    } else if (playbackState->play_time > totalDuration) {
        playbackState->play_time = totalDuration;
        playbackState->mode      = DebugPlaybackState::Mode::Paused;
    }

    interpolator_->SampleAndApply(*playbackState, playbackState->play_time, &playbackState->current_segment_index, scene);
}

void TrajectoryPlayer::SampleAtCurrentTime(const DebugPlaybackState& playbackState,
                                           omnilink::teleop_viewer::RobotScene* scene) const {
    if (scene == nullptr || !interpolator_ || playbackState.keyframes.empty()) {
        return;
    }
    int segmentIndex = -1;
    interpolator_->SampleAndApply(playbackState, playbackState.play_time, &segmentIndex, scene);
}

float TrajectoryPlayer::TotalDuration(const DebugPlaybackState& playbackState) {
    if (playbackState.keyframes.empty()) {
        return 0.0f;
    }
    return static_cast<float>(playbackState.keyframes.back().t);
}

bool TrajectoryPlayer::HasPlayableTrajectory(const DebugPlaybackState& playbackState) {
    return playbackState.keyframes.size() >= 2;
}

bool LoadTrajectoryFromCsv(const std::string& csvPath, DebugPlaybackState* playbackState, std::string* errorMessage) {
    if (playbackState == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = "playbackState is null";
        }
        return false;
    }
    std::ifstream file(csvPath);
    if (!file.good()) {
        if (errorMessage != nullptr) {
            *errorMessage = "open csv failed";
        }
        return false;
    }

    std::string line;
    std::vector<std::string> header;
    while (std::getline(file, line)) {
        const std::string stripped = kinematic_playback_internal::Trim(line);
        if (stripped.empty() || stripped[0] == '#') {
            continue;
        }
        header = kinematic_playback_internal::SplitCsvLine(stripped);
        break;
    }
    if (header.size() < 2) {
        if (errorMessage != nullptr) {
            *errorMessage = "csv header invalid, expected: time,joint1,...";
        }
        return false;
    }

    const std::string timeKey = kinematic_playback_internal::LowerString(header[0]);
    if (timeKey != "time" && timeKey != "t") {
        if (errorMessage != nullptr) {
            *errorMessage = "first csv column must be time/t";
        }
        return false;
    }

    std::vector<std::string> jointNames;
    jointNames.reserve(header.size() - 1);
    for (size_t i = 1; i < header.size(); ++i) {
        if (!header[i].empty()) {
            jointNames.push_back(header[i]);
        }
    }
    if (jointNames.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "csv joints empty";
        }
        return false;
    }

    std::vector<PoseKeyframe> loaded;
    while (std::getline(file, line)) {
        const std::string stripped = kinematic_playback_internal::Trim(line);
        if (stripped.empty() || stripped[0] == '#') {
            continue;
        }
        std::vector<std::string> cells = kinematic_playback_internal::SplitCsvLine(stripped);
        if (cells.size() < (jointNames.size() + 1)) {
            continue;
        }
        PoseKeyframe keyframe;
        try {
            keyframe.t = std::stod(cells[0]);
            for (size_t i = 0; i < jointNames.size(); ++i) {
                keyframe.joints[jointNames[i]] = static_cast<float>(std::stod(cells[i + 1]));
            }
        } catch (const std::exception&) {
            continue;
        }
        loaded.push_back(std::move(keyframe));
    }

    std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
    playbackState->keyframes                = std::move(loaded);
    playbackState->selected_keyframe_index  = playbackState->keyframes.empty() ? -1 : 0;
    playbackState->current_segment_index    = -1;
    playbackState->play_time                = 0.0f;
    playbackState->mode                     = DebugPlaybackState::Mode::Stopped;
    playbackState->timeline_edited_this_ui  = false;
    if (errorMessage != nullptr) {
        *errorMessage = "";
    }
    return !playbackState->keyframes.empty();
}

bool LoadTrajectoryFromYamlOnly(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage) {
    if (playbackState == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = "playbackState is null";
        }
        return false;
    }
    try {
        YAML::Node root       = YAML::LoadFile(path);
        YAML::Node trajectory = root["trajectory"];
        YAML::Node keyframesNode = root["keyframes"];
        if (!keyframesNode || !keyframesNode.IsSequence()) {
            keyframesNode = trajectory["keyframes"];
        }

        std::vector<PoseKeyframe> loaded;
        bool parsed = false;
        if (kinematic_playback_internal::ParseLegacyKeyframesNode(keyframesNode, &loaded)) {
            parsed = true;
        } else if (kinematic_playback_internal::ParseCompactDtValuesNode(trajectory, &loaded, errorMessage)) {
            parsed = true;
        } else if (kinematic_playback_internal::ParseCompactSamplesNode(trajectory, &loaded, errorMessage)) {
            parsed = true;
        }
        if (!parsed) {
            if (errorMessage != nullptr && errorMessage->empty()) {
                *errorMessage = "unsupported trajectory format";
            }
            return false;
        }

        playbackState->keyframes                = std::move(loaded);
        playbackState->selected_keyframe_index  = playbackState->keyframes.empty() ? -1 : 0;
        playbackState->current_segment_index    = -1;
        playbackState->play_time                = 0.0f;
        playbackState->mode                     = DebugPlaybackState::Mode::Stopped;
        playbackState->timeline_edited_this_ui  = false;
        if (errorMessage != nullptr) {
            *errorMessage = "";
        }
        return !playbackState->keyframes.empty();
    } catch (const std::exception& e) {
        if (errorMessage != nullptr) {
            *errorMessage = e.what();
        }
        return false;
    }
}

bool LoadTrajectoryFromFile(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage) {
    const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
    if (ext == ".csv") {
        return LoadTrajectoryFromCsv(path, playbackState, errorMessage);
    }
    if (ext == ".yaml" || ext == ".yml") {
        return LoadTrajectoryFromYamlOnly(path, playbackState, errorMessage);
    }
    if (errorMessage != nullptr) {
        *errorMessage = "unsupported trajectory extension: " + ext + " (expected .yaml/.yml/.csv)";
    }
    return false;
}

bool SaveTrajectoryToFile(const std::string& path, const DebugPlaybackState& playbackState, std::string* errorMessage) {
    const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
    if (ext == ".csv") {
        try {
            const std::vector<std::string> jointNames = kinematic_playback_internal::CollectJointNames(playbackState);
            std::ofstream file(path);
            if (!file.good()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "open csv failed";
                }
                return false;
            }

            file << "time";
            for (const auto& jointName : jointNames) {
                file << "," << jointName;
            }
            file << "\n";

            std::unordered_map<std::string, float> lastValues;
            for (const auto& keyframe : playbackState.keyframes) {
                file << keyframe.t;
                for (const auto& jointName : jointNames) {
                    auto it = keyframe.joints.find(jointName);
                    if (it != keyframe.joints.end()) {
                        lastValues[jointName] = it->second;
                    }
                    float value = 0.0f;
                    auto last = lastValues.find(jointName);
                    if (last != lastValues.end()) {
                        value = last->second;
                    }
                    file << "," << value;
                }
                file << "\n";
            }
            file.close();
            if (errorMessage != nullptr) {
                *errorMessage = "";
            }
            return true;
        } catch (const std::exception& e) {
            if (errorMessage != nullptr) {
                *errorMessage = e.what();
            }
            return false;
        }
    }
    if (!(ext == ".yaml" || ext == ".yml")) {
        if (errorMessage != nullptr) {
            *errorMessage = "unsupported trajectory extension: " + ext + " (expected .yaml/.yml/.csv)";
        }
        return false;
    }
    try {
        const std::vector<std::string> jointNames = kinematic_playback_internal::CollectJointNames(playbackState);
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "trajectory" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "version" << YAML::Value << 2;
        out << YAML::Key << "format" << YAML::Value << "compact";
        out << YAML::Key << "joints" << YAML::Value << YAML::BeginSeq;
        for (const auto& jointName : jointNames) {
            out << jointName;
        }
        out << YAML::EndSeq;

        double dt = 0.0;
        double t0 = 0.0;
        const bool uniformStep = kinematic_playback_internal::IsUniformTimeStep(playbackState, &dt, &t0);
        if (uniformStep) {
            out << YAML::Key << "t0" << YAML::Value << t0;
            out << YAML::Key << "dt" << YAML::Value << dt;
            out << YAML::Key << "values" << YAML::Value << YAML::BeginSeq;
            std::unordered_map<std::string, float> lastValues;
            for (const auto& keyframe : playbackState.keyframes) {
                out << YAML::BeginSeq;
                for (const auto& jointName : jointNames) {
                    auto it = keyframe.joints.find(jointName);
                    if (it != keyframe.joints.end()) {
                        lastValues[jointName] = it->second;
                    }
                    float v = 0.0f;
                    auto last = lastValues.find(jointName);
                    if (last != lastValues.end()) {
                        v = last->second;
                    }
                    out << v;
                }
                out << YAML::EndSeq;
            }
            out << YAML::EndSeq;
        } else {
            out << YAML::Key << "samples" << YAML::Value << YAML::BeginSeq;
            std::unordered_map<std::string, float> lastValues;
            for (const auto& keyframe : playbackState.keyframes) {
                out << YAML::BeginSeq;
                out << keyframe.t;
                for (const auto& jointName : jointNames) {
                    auto it = keyframe.joints.find(jointName);
                    if (it != keyframe.joints.end()) {
                        lastValues[jointName] = it->second;
                    }
                    float v = 0.0f;
                    auto last = lastValues.find(jointName);
                    if (last != lastValues.end()) {
                        v = last->second;
                    }
                    out << v;
                }
                out << YAML::EndSeq;
            }
            out << YAML::EndSeq;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;

        std::ofstream file(path);
        if (!file.good()) {
            if (errorMessage != nullptr) {
                *errorMessage = "open file failed";
            }
            return false;
        }
        file << out.c_str();
        file.close();
        if (errorMessage != nullptr) {
            *errorMessage = "";
        }
        return true;
    } catch (const std::exception& e) {
        if (errorMessage != nullptr) {
            *errorMessage = e.what();
        }
        return false;
    }
}

bool LoadTrajectoryFromYaml(const std::string& yamlPath, DebugPlaybackState* playbackState, std::string* errorMessage) {
    return LoadTrajectoryFromYamlOnly(yamlPath, playbackState, errorMessage);
}

bool SaveTrajectoryToYaml(const std::string& yamlPath, const DebugPlaybackState& playbackState, std::string* errorMessage) {
    return SaveTrajectoryToFile(yamlPath, playbackState, errorMessage);
}

void BuildDemoTrajectoryFromCurrentPose(DebugPlaybackState* playbackState,
                                        const std::vector<omnilink::teleop_viewer::RobotScene::JointInfo>& joints) {
    if (playbackState == nullptr) {
        return;
    }
    playbackState->keyframes.clear();
    playbackState->mode                  = DebugPlaybackState::Mode::Stopped;
    playbackState->play_time             = 0.0f;
    playbackState->selected_keyframe_index = -1;
    playbackState->current_segment_index = -1;

    std::vector<omnilink::teleop_viewer::RobotScene::JointInfo> revoluteJoints;
    revoluteJoints.reserve(joints.size());
    for (const auto& joint : joints) {
        if (joint.revolute) {
            revoluteJoints.push_back(joint);
        }
    }
    if (revoluteJoints.empty()) {
        return;
    }

    const int frameCount = 16;
    const float dt       = std::max(0.1f, playbackState->keyframe_interval_sec);
    const float twoPi    = 6.283185307f;
    for (int frame = 0; frame < frameCount; ++frame) {
        PoseKeyframe keyframe;
        keyframe.t = static_cast<double>(frame) * static_cast<double>(dt);
        const float phase = kinematic_playback_internal::WrapPhase((static_cast<float>(frame) / static_cast<float>(frameCount - 1)) * twoPi);
        for (size_t i = 0; i < revoluteJoints.size(); ++i) {
            const auto& joint = revoluteJoints[i];
            const float jointRange = std::max(0.0f, joint.max_angle - joint.min_angle);
            const float amplitudeByRange = std::max(0.03f, std::min(0.25f, 0.12f * jointRange));
            const float amplitude        = std::min(amplitudeByRange, 0.35f);
            const float offsetPhase      = phase + static_cast<float>(i) * 0.35f;
            float value                  = joint.position + amplitude * std::sin(offsetPhase);
            value                        = std::clamp(value, joint.min_angle, joint.max_angle);
            keyframe.joints[joint.name]  = value;
        }
        playbackState->keyframes.push_back(std::move(keyframe));
    }

    playbackState->selected_keyframe_index = 0;
}

}  // namespace kinematic_viewer
