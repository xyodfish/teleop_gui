#include "config_ui.h"
#include <filesystem>
#include <string>
#include <vector>
#include "config_parser.h"
#include "imgui.h"

namespace ConfigUI {

    ConfigViewer::ConfigViewer() {
        // 设置默认搜索路径
        search_paths_ = {"/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/", "/home/yuxia/Workspace/SingoriX/Deploy/",
                         "~/Workspace/SingoriX/"};

        window_pos_  = ImVec2(100, 100);
        window_size_ = ImVec2(800, 600);
        is_visible_  = false;  // 默认不显示
    }

    void ConfigViewer::setVisible(bool visible) { is_visible_ = visible; }
    bool ConfigViewer::isVisible() const { return is_visible_; }

    void ConfigViewer::setConfigPath(const std::string& path) { config_path_ = path; }

    void ConfigViewer::show() {
        if (ImGui::Button("显示/隐藏配置配置文件窗口")) {
            setVisible(!isVisible());
        }

        if (!is_visible_)
            return;

        ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);

        // 禁用窗口移动和调整大小
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse;

        bool window_open = true;
        if (ImGui::Begin("SingoriX Omnilink 配置", &window_open, flags)) {
            showMenuBar();
            showFileSelector();
            showConfigTree();
        }

        // 如果用户点击了关闭按钮
        if (!window_open) {
            is_visible_ = false;
        }

        ImGui::End();
    }

    void ConfigViewer::showMenuBar() {
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("文件")) {
                if (ImGui::MenuItem("重新加载")) {
                    loadConfig();
                }
                if (ImGui::MenuItem("选择文件...")) {
                    show_file_dialog_ = true;
                }
                if (ImGui::MenuItem("保存配置")) {
                    // 这里可以添加保存功能
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
    }

    // 新增：文件选择器界面
    void ConfigViewer::showFileSelector() {
        ImGui::Text("当前配置文件:");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.8f, 1.0f), "%s", config_path_.c_str());

        ImGui::SameLine();
        if (ImGui::Button("选择文件")) {
            show_file_dialog_ = true;
        }

        ImGui::SameLine();
        if (ImGui::Button("重新加载")) {
            loadConfig();
        }

        ImGui::Separator();

        // 文件选择对话框
        if (show_file_dialog_) {
            showFileDialog();
        }
    }

    // 新增：文件选择对话框
    void ConfigViewer::showFileDialog() {
        ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_Appearing);
        if (ImGui::Begin("选择配置文件", &show_file_dialog_)) {
            static char path_input[256] = "";

            // 快速路径输入
            ImGui::Text("直接输入路径:");
            ImGui::InputText("##path_input", path_input, sizeof(path_input));
            ImGui::SameLine();
            if (ImGui::Button("确认") && path_input[0] != '\0') {
                config_path_ = path_input;
                loadConfig();
                show_file_dialog_ = false;
            }

            ImGui::Separator();
            ImGui::Text("常用路径:");

            // 显示常用路径
            for (const auto& base_path : search_paths_) {
                if (ImGui::TreeNode(base_path.c_str())) {
                    try {
                        std::filesystem::path fs_path(base_path);
                        if (std::filesystem::exists(fs_path)) {
                            for (const auto& entry : std::filesystem::recursive_directory_iterator(fs_path)) {
                                if (entry.is_regular_file() &&
                                    (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml")) {
                                    std::string full_path = entry.path().string();
                                    if (ImGui::Selectable(full_path.c_str())) {
                                        config_path_ = full_path;
                                        loadConfig();
                                        show_file_dialog_ = false;
                                    }
                                }
                            }
                        }
                    } catch (const std::exception& e) { ImGui::Text("无法访问路径: %s", e.what()); }
                    ImGui::TreePop();
                }
            }

            // 当前目录文件浏览
            ImGui::Separator();
            ImGui::Text("浏览文件:");

            static std::string current_dir = ".";
            static std::vector<std::string> dir_history;

            ImGui::Text("当前目录: %s", current_dir.c_str());

            if (ImGui::Button("上一级") && current_dir != "." && current_dir != "/") {
                dir_history.push_back(current_dir);
                current_dir = std::filesystem::path(current_dir).parent_path().string();
            }

            ImGui::SameLine();
            if (ImGui::Button("刷新")) {
                // 刷新目录
            }

            try {
                for (const auto& entry : std::filesystem::directory_iterator(current_dir)) {
                    std::string name = entry.path().filename().string();
                    bool is_dir      = entry.is_directory();

                    if (ImGui::Selectable(name.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick)) {
                        if (ImGui::IsMouseDoubleClicked(0)) {
                            if (is_dir) {
                                dir_history.push_back(current_dir);
                                current_dir = entry.path().string();
                            } else if (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml") {
                                config_path_ = entry.path().string();
                                loadConfig();
                                show_file_dialog_ = false;
                            }
                        }
                    }

                    if (is_dir) {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), " [目录]");
                    } else if (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml") {
                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), " [YAML]");
                    }
                }
            } catch (const std::exception& e) { ImGui::Text("错误: %s", e.what()); }
        }
        ImGui::End();
    }

    void ConfigViewer::loadConfig() {
        config_data_.clear();
        ConfigParser::YamlConfig parser;
        if (parser.loadFromFile(config_path_)) {
            config_data_ = parser.parseToTree();
        }
    }

    void ConfigViewer::showConfigTree() {
        if (config_data_.empty()) {
            ImGui::Text("请选择一个配置文件");
            return;
        }

        ImGui::BeginChild("ConfigTree", ImVec2(0, 0), true);
        for (auto& item : config_data_) {
            showConfigItem(item);
        }
        ImGui::EndChild();
    }

    void ConfigViewer::showConfigItem(const ConfigParser::ConfigValue& item) {
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_DefaultOpen;

        if (item.children.empty()) {
            flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_Bullet;
        }

        bool isOpen = ImGui::TreeNodeEx(item.key.c_str(), flags);

        if (item.type == "value") {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), ": %s", item.value.c_str());
        } else {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), " (%s)", item.type.c_str());
        }

        if (isOpen) {
            for (const auto& child : item.children) {
                showConfigItem(child);
            }
            ImGui::TreePop();
        }
    }

}  // namespace ConfigUI