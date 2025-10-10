// Dear ImGui: 简化版 GLFW + OpenGL 3 示例

#include <stdio.h>
#include "config_ui.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "launcher_module_ui.h"
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <cstdlib>

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

ConfigUI::ConfigViewer config_viewer;

void Show() {
    // 添加一个按钮来控制配置查看器的显示/隐藏
    LauncherModuleUI::Show();
    config_viewer.show();
}

// Main code
int main(int, char**) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // 创建窗口
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Galbot SingoriX OmniLink Teleop Window", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // 启用垂直同步

    // 初始化 Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // 启用键盘控制
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;      // 启用停靠
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;    // 启用多视口

    io.Fonts->AddFontDefault();
    ImFontConfig config;
    config.MergeMode        = true;   // 合并模式，支持中文
    config.GlyphMinAdvanceX = 13.0f;  // 如果字符太宽，使用这个

    // 指定中文字体文件路径
    static const ImWchar icons_ranges[] = {0x4e00, 0x9fff, 0};  // 中文范围

    // 尝试加载系统中文字体
    io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", 16.0f, &config, icons_ranges);

    // 设置 Dear ImGui 样式
    ImGui::StyleColorsDark();

    // 当启用视口时，调整窗口样式
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiStyle& style                 = ImGui::GetStyle();
        style.WindowRounding              = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // 初始化平台/渲染器后端
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // 应用状态
    bool show_demo_window    = true;
    bool show_another_window = false;
    ImVec4 clear_color       = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // 主循环
    while (!glfwWindowShouldClose(window)) {
        // 处理事件
        glfwPollEvents();

        // 开始 Dear ImGui 帧
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        Show();

        // 渲染
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // 更新和渲染额外的平台窗口
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }

    // 清理
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}