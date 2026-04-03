#include "teleop_viewer/app.h"
#include "teleop_viewer/config.h"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string config_path = "config/robot_viewer.yaml";
    if (argc > 1 && argv[1] != nullptr) {
        config_path = argv[1];
    }

    bool loaded_ok = false;
    auto config = omnilink::teleop_viewer::ViewerConfig::LoadFromFile(config_path, &loaded_ok);

    if (loaded_ok) {
        std::cout << "[robot_viewer] Config loaded: " << config_path << std::endl;
    } else {
        std::cout << "[robot_viewer] Using built-in defaults. (Config path: " << config_path << ")" << std::endl;
    }

    omnilink::teleop_viewer::RobotViewerApp app(config);
    return app.run();
}
