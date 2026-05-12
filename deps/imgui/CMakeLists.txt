file(GLOB IMGUI_SOURCES "*.cpp")

find_package(OpenGL REQUIRED)

# 动态库：产物为 libimgui.so，输出目录由顶层 CMAKE_LIBRARY_OUTPUT_DIRECTORY 决定
add_library(imgui SHARED ${IMGUI_SOURCES})
target_link_libraries(imgui PRIVATE glfw OpenGL::GL dl)
