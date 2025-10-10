#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include "config_parser.h"
#include "imgui.h"

namespace ConfigUI {

    class ConfigViewer {
       public:
        ConfigViewer();

        void setConfigPath(const std::string& path);

        void show();

        void setVisible(bool visible);
        bool isVisible() const;

       private:
        void showMenuBar();

        // 新增：文件选择器界面
        void showFileSelector();

        // 新增：文件选择对话框
        void showFileDialog();
        void loadConfig();

        void showConfigTree();

        void showConfigItem(const ConfigParser::ConfigValue& item);

        std::string config_path_;
        std::vector<ConfigParser::ConfigValue> config_data_;
        std::vector<std::string> search_paths_;
        bool show_file_dialog_ = false;

        ImVec2 window_pos_;
        ImVec2 window_size_;
        bool is_visible_;
    };

}  // namespace ConfigUI