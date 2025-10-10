file(GLOB IMGUI_SOURCES "*.cpp")

find_package(OpenGL REQUIRED)


# 创建库并添加所有源文件
add_library(imgui ${IMGUI_SOURCES})
target_link_libraries(imgui glfw OpenGL::GL dl)

