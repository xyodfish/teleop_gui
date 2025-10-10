#include "launcher_module_ui.h"
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include "imgui.h"
#include "spdlog/spdlog.h"

namespace LauncherModuleUI {
    // 跟踪启动状态的结构
    struct ProcessStatus {
        std::string process_name;
        std::string name_label;
        bool is_launched = false;
        std::chrono::steady_clock::time_point launch_time;
        bool confirmed_running = false;
        std::string log_content;
        bool show_log_window = false;
        FILE* process_pipe   = nullptr;  // 管道文件指针
        std::thread log_thread;          // 日志读取线程
        std::mutex log_mutex;            // 保护日志内容的互斥锁
        bool should_stop = false;        // 停止标志

        // 构造函数
        ProcessStatus() = default;
        ProcessStatus(const std::string& pName, const std::string& label) : process_name(pName), name_label(label) {}

        ~ProcessStatus() { stopLogging(); }

        void stopLogging() {
            if (process_pipe) {
                pclose(process_pipe);
                process_pipe = nullptr;
            }
            should_stop = true;
            if (log_thread.joinable()) {
                log_thread.join();
            }
        }
    };

    static ProcessStatus simulator_status("~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/bin/singorix_mujoco_sim",
                                          "机器人仿真:");
    static ProcessStatus wbc_status("~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/bin/singorix_wbcs_main", "WBC:");
    static ProcessStatus teleop_status("~/Workspace/SingoriX/OmniLink/singorix_omnilink/bin/singorix_omnilink", "遥操:");

    // 检查进程是否正在运行
    bool isProcessRunning(const std::string& process_name) {
        std::string command = "pgrep -f " + process_name + " > /dev/null 2>&1";
        return system(command.c_str()) == 0;
    }

    // 读取管道中的日志内容
    void readProcessOutput(ProcessStatus& status, const std::string& command) {
        status.process_pipe = popen(command.c_str(), "r");
        if (!status.process_pipe) {
            std::lock_guard<std::mutex> lock(status.log_mutex);
            status.log_content += "无法启动进程: " + command + "\n";
            return;
        }

        char buffer[1024];
        while (!status.should_stop && fgets(buffer, sizeof(buffer), status.process_pipe) != nullptr) {
            std::lock_guard<std::mutex> lock(status.log_mutex);
            status.log_content += buffer;
            // 限制日志大小以避免内存问题
            if (status.log_content.size() > 100000) {  // 100KB
                status.log_content = status.log_content.substr(status.log_content.size() - 50000);
            }
        }
    }

    void updateRunningStatus(ProcessStatus& status) {
        const char* name_label = status.name_label.c_str();
        ImGui::TextUnformatted(name_label);
        ImGui::SameLine();
        if (status.is_launched) {
            if (status.confirmed_running) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "运行中");
            } else {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "启动中...");
            }
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "未运行");
        }

        // 显示日志按钮
        ImGui::SameLine();
        std::string button_label = "日志##" + status.name_label;
        if (ImGui::SmallButton(button_label.c_str())) {
            status.show_log_window = !status.show_log_window;
        }
    }

    // 显示日志窗口
    void showLogWindow(ProcessStatus& status) {
        if (status.show_log_window) {
            std::string window_title = status.name_label + " 日志###" + status.name_label + "_log";
            ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
            if (ImGui::Begin(window_title.c_str(), &status.show_log_window)) {
                std::lock_guard<std::mutex> lock(status.log_mutex);

                // 创建一个可滚动的文本区域
                if (ImGui::BeginChild("LogScroll", ImVec2(0, 0), true)) {
                    ImGui::TextUnformatted(status.log_content.c_str());
                    // 自动滚动到底部
                    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
                        ImGui::SetScrollHereY(1.0f);
                    }
                }
                ImGui::EndChild();
            }
            ImGui::End();
        }
    }

    // 检查并更新进程状态
    void updateProcessStatus(ProcessStatus& status) {
        auto elapsed      = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - status.launch_time).count();
        auto process_name = status.process_name;
        if (status.is_launched) {
            // 检查进程是否已经启动（给一点时间）

            if (!status.confirmed_running) {
                if (elapsed > 2) {
                    status.confirmed_running = isProcessRunning(process_name);
                }
            } else {
                // 如果进程已启动
                if (elapsed > 0.5) {
                    status.confirmed_running = isProcessRunning(process_name);
                    if (!status.confirmed_running) {
                        spdlog::info("Process {} stopped", process_name);
                        status.is_launched = false;
                        status.stopLogging();
                    }
                }
            }
        } else {
            if (!status.confirmed_running) {
                if (elapsed > 2) {
                    status.confirmed_running = isProcessRunning(process_name);
                    if (status.confirmed_running) {
                        spdlog::info("Process {} started", process_name);
                        status.is_launched = true;
                        status.stopLogging();
                    }
                }
            }
        }
    }

    void Show() {
        ImGui::SetNextWindowSize(ImVec2(400, 400), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_FirstUseEver);

        ImGui::Begin("启动模块", NULL, ImGuiWindowFlags_None);

        // 更新所有进程状态
        updateProcessStatus(simulator_status);
        updateProcessStatus(wbc_status);
        updateProcessStatus(teleop_status);

        // 显示详细状态
        ImGui::Text("程序状态:");
        ImGui::Separator();

        updateRunningStatus(simulator_status);
        updateRunningStatus(wbc_status);
        updateRunningStatus(teleop_status);

        ImGui::Separator();

        if (ImGui::Button("打开机器人仿真")) {
            // 停止之前的日志线程
            simulator_status.stopLogging();

            // 启动带管道的进程
            std::string cmd =
                std::string("nohup ~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/bin/singorix_mujoco_sim ") +
                "~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/share/galbot/singorix_simulator/model/galbot/" +
                "galbot_one_charlie/galbot_one_charlie-effort.xml 2>&1";

            simulator_status.should_stop = false;
            simulator_status.log_thread  = std::thread(readProcessOutput, std::ref(simulator_status), cmd);

            simulator_status.is_launched       = true;
            simulator_status.launch_time       = std::chrono::steady_clock::now();
            simulator_status.confirmed_running = false;
        }

        if (ImGui::Button("打开WBC")) {
            // 停止之前的日志线程
            wbc_status.stopLogging();

            // 启动带管道的进程
            std::string cmd =
                std::string("nohup ~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/bin/singorix_wbcs_main ") +
                "~/Workspace/SingoriX/Deploy/singorix_release/x86_64-Linux-GNU-9.4.0/config/SingoriX/galbot_one_charlie/mujoco/" +
                "robot_config.toml 2>&1";

            wbc_status.should_stop = false;
            wbc_status.log_thread  = std::thread(readProcessOutput, std::ref(wbc_status), cmd);

            wbc_status.is_launched       = true;
            wbc_status.launch_time       = std::chrono::steady_clock::now();
            wbc_status.confirmed_running = false;
        }

        if (ImGui::Button("打开遥操")) {
            // 停止之前的日志线程
            teleop_status.stopLogging();

            // 启动带管道的进程
            std::string cmd =
                std::string("nohup ~/Workspace/SingoriX/OmniLink/singorix_omnilink/bin/singorix_omnilink omnilink_for_mujoco.yaml 2>&1");

            teleop_status.should_stop = false;
            teleop_status.log_thread  = std::thread(readProcessOutput, std::ref(teleop_status), cmd);

            teleop_status.is_launched       = true;
            teleop_status.launch_time       = std::chrono::steady_clock::now();
            teleop_status.confirmed_running = false;
        }

        // 关闭按钮
        if (ImGui::Button("关闭机器人仿真")) {
            std::system("pkill -f singorix_mujoco_sim");
            simulator_status.is_launched       = false;
            simulator_status.confirmed_running = false;
            simulator_status.stopLogging();
        }

        if (ImGui::Button("关闭WBC")) {
            std::system("pkill -f singorix_wbcs_main");
            wbc_status.is_launched       = false;
            wbc_status.confirmed_running = false;
            wbc_status.stopLogging();
        }

        if (ImGui::Button("关闭遥操")) {
            std::system("pkill -f singorix_omnilink");
            teleop_status.is_launched       = false;
            teleop_status.confirmed_running = false;
            teleop_status.stopLogging();
        }

        ImGui::End();

        // 显示日志窗口
        showLogWindow(simulator_status);
        showLogWindow(wbc_status);
        showLogWindow(teleop_status);
    }
}  // namespace LauncherModuleUI