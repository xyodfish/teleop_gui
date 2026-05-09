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

if (NOT COMMON_LIB_ROOT_PATH)
    set(COMMON_LIB_ROOT_PATH /opt/galbot/devel/)
endif ()

# 多平台部署
if(NOT ARCH_PREFIX)
        set(ARCH_PREFIX ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION})
endif()


# 设定默认安装路径
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../../devel/${ARCH_FREFIX} CACHE PATH "Default installation directory" FORCE)
endif()

# 设置三方库开发平台路径
if (NOT ROOT_DEVEL_ARCH_PATH)
    set(ROOT_DEVEL_ARCH_PATH /opt/galbot/devel/${ARCH_FREFIX})
endif ()
if (NOT CONTROL_DEVEL_ARCH_PATH)
    set(CONTROL_DEVEL_ARCH_PATH /opt/galbot/devel_control/${ARCH_FREFIX})
endif ()
cmake_print_variables(CMAKE_INSTALL_PREFIX)

# 设定默认三方库配置文件查找路径
list(APPEND CMAKE_PREFIX_PATH 0 ${CMAKE_INSTALL_PREFIX})
list(APPEND CMAKE_PREFIX_PATH ${ROOT_DEVEL_ARCH_PATH})
list(APPEND CMAKE_PREFIX_PATH ${CONTROL_DEVEL_ARCH_PATH})
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
find_package(pinocchio REQUIRED)
find_package(tinyxml2 QUIET)
find_package(qpOASES QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ROSCPP REQUIRED IMPORTED_TARGET roscpp)

if (NOT qpOASES_FOUND)
    find_library(QPOASES_LIBRARY NAMES qpOASES
                 PATHS /opt/galbot/devel_control/${ARCH_FREFIX}/lib
                       /opt/galbot/devel_control/${ARCH_PREFIX}/lib
                       /opt/galbot/devel_control/x86_64-Linux-GNU-9.4.0/lib)
endif()

# Some trac_ik exports expect tinyxml2::tinyxml2; define a compatible target when missing.
if (TARGET TinyXML2::TinyXML2 AND NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 ALIAS TinyXML2::TinyXML2)
endif()
if (NOT TARGET tinyxml2::tinyxml2)
    find_library(TINYXML2_LIBRARY NAMES tinyxml2)
    if (TINYXML2_LIBRARY)
        add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
        set_target_properties(tinyxml2::tinyxml2 PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIBRARY}")
    endif()
endif()


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
    /opt/ros/noetic/include
    ${singorix_proto_INCLUDE_DIRS}
    ${glog_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${EMBOSA_INCLUDE_DIR}
    ${trac_ik_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include/ik_backends/flex_ik
    ${PROJECT_SOURCE_DIR}/include/ik_backends/wbc_chain_ik
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

add_library(imguizmo
    deps/imguizmo/ImGuizmo.cpp
)

add_library(flex_ik_core STATIC
    src/ik_backends/flex_ik/flex_ik.cc
)
target_include_directories(flex_ik_core PUBLIC include/ik_backends/flex_ik)
target_link_libraries(flex_ik_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)

add_library(wbc_chain_ik_core STATIC
    src/ik_backends/wbc_chain_ik/robot/traits/chain_ik.cpp
    src/ik_backends/wbc_chain_ik/solver/hqp_solver.cpp
    src/ik_backends/wbc_chain_ik/solver/qp_solver.cpp
    src/ik_backends/wbc_chain_ik/math/helper_functions.cpp
)
target_include_directories(wbc_chain_ik_core PUBLIC include/ik_backends/wbc_chain_ik)
target_link_libraries(wbc_chain_ik_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)

# 将你的库与必要的依赖项链接：ImGui本身、GLFW、OpenGL
target_link_libraries(imgui_glfw_support PUBLIC imgui glfw OpenGL::GL)
# 包含必要的头文件目录
target_include_directories(imgui_glfw_support PUBLIC deps/imgui)
target_include_directories(imguizmo PUBLIC deps/imgui deps/imguizmo)

add_executable(test_gui src/test_gui.cpp)
add_executable(test_demo src/test_demo.cpp src/launcher_module_ui.cpp src/config_ui.cpp )

# add_executable(robot_viewer src/robot_viewer.cpp deps/glad/glad.c)
add_executable(robot_viewer
    src/robot_viewer_copy_2.cpp
    src/teleop_viewer/app.cpp
    src/teleop_viewer/app_sidebar.cpp
    src/teleop_viewer/config.cpp
    src/teleop_viewer/msg_publisher.cpp
    src/teleop_viewer/scene.cpp
    src/teleop_viewer/sensor_subscriber.cpp
    deps/glad/glad.c
)

add_executable(robot_kinematic_viewer
    src/robot_kinematic_viewer.cpp
    src/kinematic_viewer/kinematic_viewer_config.cpp
    src/kinematic_viewer/kinematic_bootstrap.cpp
    src/kinematic_viewer/kinematic_playback.cpp
    src/kinematic_viewer/kinematic_collision_monitor.cpp
    src/kinematic_viewer/kinematic_marker_target_state.cpp
    src/kinematic_viewer/kinematic_ros_bridge.cpp
    src/kinematic_viewer/kinematic_sidebar_panels.cpp
    src/kinematic_viewer/kinematic_ui_theme.cpp
    src/kinematic_viewer/kinematic_marker_utils.cpp
    src/kinematic_viewer/kinematic_shader_utils.cpp
    src/kinematic_viewer/kinematic_line_renderer.cpp
    src/teleop_viewer/ik_solver.cpp
    src/teleop_viewer/scene.cpp
    deps/glad/glad.c
)

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
    yaml-cpp

)

target_link_libraries(robot_kinematic_viewer
    imgui_glfw_support
    imguizmo
    assimp
    urdfdom_model
    urdfdom_world
    urdfdom_sensor
    ${trac_ik_LIBRARIES}
    flex_ik_core
    wbc_chain_ik_core
    OpenGL::GL
    glfw
    GLEW::GLEW
    PkgConfig::ROSCPP
    pthread
    yaml-cpp
)

if (qpOASES_FOUND)
    target_link_libraries(robot_kinematic_viewer ${qpOASES_LIBRARIES})
elseif (QPOASES_LIBRARY)
    target_link_libraries(robot_kinematic_viewer ${QPOASES_LIBRARY})
else()
    message(WARNING "qpOASES not explicitly linked; full-body IK backends may fail to link/runtime.")
endif()


# 最后，将上面创建的库链接到你的主程序目标（在src/CMakeLists.txt中定义）
target_link_libraries(test_gui PRIVATE imgui_glfw_support)
target_link_libraries(test_demo PRIVATE imgui_glfw_support pthread yaml-cpp)


# 复制着色器文件
file(COPY shader DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
