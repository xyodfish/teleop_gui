cmake_minimum_required(VERSION 3.16)

# 项目名称和版本号，必填，版本号用于版本管理
project(singorix_teleop_gui VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The target C++ standard.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 打印助手
include(CMakePrintHelpers)
set(CMAKE_BUILD_TYPE Debug) # Debug

if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    add_compile_options(-g)
endif()


# 多平台部署
if (NOT ARCH_FREFIX)
    set(ARCH_FREFIX ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION})
endif ()

# 默认使用Release构建类型
if ( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
    add_compile_options(-g)
    message( STATUS "Setting build type to '${CMAKE_BUILD_TYPE}'" )
else ( NOT CMAKE_BUILD_TYPE )
    message( STATUS "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
endif ( NOT CMAKE_BUILD_TYPE )

# 设定默认安装路径
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../../devel/${ARCH_FREFIX} CACHE PATH "Default installation directory" FORCE)
endif()

# 设置三方库开发平台路径
if (NOT ROOT_DEVEL_ARCH_PATH)
    set(ROOT_DEVEL_ARCH_PATH /opt/galbot/devel/${ARCH_FREFIX})
endif ()
cmake_print_variables(CMAKE_INSTALL_PREFIX)

# 设定默认三方库配置文件查找路径
list(APPEND CMAKE_PREFIX_PATH 0 ${CMAKE_INSTALL_PREFIX})
list(APPEND CMAKE_PREFIX_PATH ${ROOT_DEVEL_ARCH_PATH})
cmake_print_variables(CMAKE_PREFIX_PATH)
# 打印第三方库配置文件寻找过程
set(CMAKE_FIND_DEBUG_MODE 0)


# 查找依赖
find_package(glog REQUIRED)
find_package(trac_ik REQUIRED)
find_package(singorix_proto REQUIRED)
find_package(omnilink_proto REQUIRED)
find_package(singorix_common REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenGL REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(GLEW REQUIRED)


get_target_property(glog_INCLUDE_DIRS glog::glog INTERFACE_INCLUDE_DIRECTORIES)

set(EMBOSA_INCLUDE_DIR ${ROOT_DEVEL_ARCH_PATH}/include/embosa)
set(EMBOSA_LIBRARY_DIR ${ROOT_DEVEL_ARCH_PATH}/lib)
set(EMBOSA_LIBRARIES embosa fastcdr fastrtps foonathan_memory-0.7.3 ssl crypto tinyxml2 spdlog protobuf z rt dl boost_thread)


cmake_print_variables(EMBOSA_INCLUDE_DIR)


cmake_print_variables(trac_ik_LIBRARIES)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(INC_DIR ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(THIRD_PARTY_LIB ${PROJECT_SOURCE_DIR}/3rd/lib)
set(THIRD_PARTY_INC ${PROJECT_SOURCE_DIR}/3rd/include)


# 下面是自定义的构建
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/deps/imgui
    ${PROJECT_SOURCE_DIR}/3rd/include
    ${INC_DIR}
    ${singorix_proto_INCLUDE_DIRS}
    ${glog_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${EMBOSA_INCLUDE_DIR}
    ${trac_ik_INCLUDE_DIR}
)

link_directories(
    ${PROJECT_SOURCE_DIR}/3rd/lib
    ${EMBOSA_LIBRARY_DIR}
    ${omnilink_proto_LIBRARY_DIRS}
)




add_subdirectory(deps/imgui)

add_library(imgui_glfw_support
    deps/imgui/imgui_impl_glfw.cpp
    deps/imgui/imgui_impl_opengl3.cpp
)

# 将你的库与必要的依赖项链接：ImGui本身、GLFW、OpenGL
target_link_libraries(imgui_glfw_support PUBLIC imgui glfw OpenGL::GL)
# 包含必要的头文件目录
target_include_directories(imgui_glfw_support PUBLIC deps/imgui)

add_executable(test_gui src/test_gui.cpp)
add_executable(test_demo src/test_demo.cpp src/launcher_module_ui.cpp src/config_ui.cpp )

# add_executable(robot_viewer src/robot_viewer.cpp deps/glad/glad.c)
add_executable(robot_viewer src/robot_viewer_copy_2.cpp deps/glad/glad.c)

# 链接库
target_link_libraries(robot_viewer
    imgui_glfw_support
    assimp
    urdfdom_model
    urdfdom_world
    urdfdom_sensor
    OpenGL::GL
    glfw
    GLEW::GLEW
    singorix_proto::singorix_proto
    omnilink_proto::omnilink_proto
    ${EMBOSA_LIBRARIES}
    pthread

)


# 最后，将上面创建的库链接到你的主程序目标（在src/CMakeLists.txt中定义）
target_link_libraries(test_gui PRIVATE imgui_glfw_support)
target_link_libraries(test_demo PRIVATE imgui_glfw_support pthread yaml-cpp)


# 复制着色器文件
file(COPY shader DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
