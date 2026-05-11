#pragma once

#include "kinematic_viewer/kinematic_viewer_config.h"

#include <string>
#include <vector>

namespace kinematic_viewer {

void SetupKinematicViewerFonts(const KinematicViewerConfig& cfg);
const std::vector<const char*>& KinematicUiThemeNames();
int KinematicUiThemeIndexFromName(const std::string& themeName);
void ApplyKinematicUiStyleByIndex(int themeIndex);
void ApplyKinematicUiStyleByName(const std::string& themeName);

}  // namespace kinematic_viewer
