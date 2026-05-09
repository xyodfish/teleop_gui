#include "kinematic_viewer/kinematic_bootstrap.h"
#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_viewer_config.h"
#include "kinematic_viewer/kinematic_marker_utils.h"
#include "kinematic_viewer/kinematic_line_renderer.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "kinematic_viewer/kinematic_marker_target_state.h"
#include "kinematic_viewer/kinematic_shader_utils.h"
#include "kinematic_viewer/kinematic_ui_theme.h"
#include "teleop_viewer/ik_solver.h"
#include "teleop_viewer/scene.h"
#include "kinematic_viewer/kinematic_ros_bridge.h"

#include "imgui.h"
#include "ImGuizmo.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <geometry_msgs/PoseStamped.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using omnilink::teleop_viewer::IkSolveStats;
using kinematic_viewer::DebugPlaybackState;
using kinematic_viewer::CollisionMonitor;
using kinematic_viewer::CollisionMonitorResult;
using kinematic_viewer::CollisionMonitorState;
using kinematic_viewer::IkState;
using kinematic_viewer::KinematicLineRenderer;
using kinematic_viewer::KinematicLineVertex;
using kinematic_viewer::KinematicRosBridge;
using kinematic_viewer::KinematicViewerConfig;
using kinematic_viewer::LoadActiveMarkerFromTarget;
using kinematic_viewer::RenderJointPanel;
using kinematic_viewer::RenderPlaybackPanel;
using kinematic_viewer::RenderScenePanel;
using kinematic_viewer::RenderSafetyPanel;
using kinematic_viewer::RenderTfPanel;
using kinematic_viewer::SaveActiveMarkerToTarget;
using kinematic_viewer::ViewerState;
using omnilink::teleop_viewer::OrbitCamera;
using omnilink::teleop_viewer::RobotScene;
using kinematic_viewer::TrajectoryPlayer;
using kinematic_viewer::appendCircle;
using kinematic_viewer::appendMarkerAxes;
using kinematic_viewer::createKinematicLineProgram;
using kinematic_viewer::createKinematicMeshProgram;
using kinematic_viewer::distancePointToSegment2D;
using kinematic_viewer::EnsureMarkerTargetInitialized;
using kinematic_viewer::LaunchConfig;
using kinematic_viewer::LoadLaunchConfigFromArgs;
using kinematic_viewer::markerWorldMatrix;
using kinematic_viewer::worldToScreen;
using kinematic_viewer::wrapDeltaDeg;

namespace robot_kinematic_viewer_internal {

    float g_scroll_delta = 0.0f;

    void ScrollCallback(GLFWwindow*, double, double yoffset) {
        g_scroll_delta += static_cast<float>(yoffset);
    }

}  // namespace robot_kinematic_viewer_internal

using robot_kinematic_viewer_internal::g_scroll_delta;
using robot_kinematic_viewer_internal::ScrollCallback;

int main(int argc, char** argv) {
    LaunchConfig launch;
    std::string launchError;
    if (!LoadLaunchConfigFromArgs(argc, argv, &launch, &launchError)) {
        if (!launchError.empty()) {
            std::cerr << launchError << std::endl;
        }
        return 1;
    }
    KinematicViewerConfig cfg = launch.config;
    std::string urdf_path     = launch.urdfPath.empty() ? cfg.robot.urdf_path : launch.urdfPath;

    if (!glfwInit()) {
        std::cerr << "glfwInit failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(cfg.window.width, cfg.window.height, "Robot Kinematic Debug Viewer", nullptr, nullptr);
    if (!window) {
        std::cerr << "create window failed\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetScrollCallback(window, ScrollCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "glad init failed\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    kinematic_viewer::SetupKinematicViewerFonts(cfg);
    kinematic_viewer::ApplyKinematicUiStyle();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    GLuint mesh_shader = createKinematicMeshProgram();
    GLuint line_shader = createKinematicLineProgram();
    KinematicLineRenderer line_renderer;
    line_renderer.init();

    RobotScene scene;
    if (!scene.loadURDF(urdf_path)) {
        std::cerr << "Failed to load URDF: " << urdf_path << "\n";
        return 1;
    }

    OrbitCamera camera;
    camera.distance     = cfg.camera.distance;
    camera.yaw          = cfg.camera.yaw;
    camera.pitch        = cfg.camera.pitch;
    camera.target       = cfg.camera.target;
    camera.rotate_speed = cfg.camera.rotate_speed;
    camera.zoom_scale   = cfg.camera.zoom_scale;
    camera.dolly_scale  = cfg.camera.dolly_scale;
    camera.pan_scale    = cfg.camera.pan_scale;
    camera.min_distance = cfg.camera.min_distance;
    camera.max_distance = cfg.camera.max_distance;

    ViewerState ui_state;
    ui_state.lock_base = cfg.ui.fix_base_like_mujoco;
    scene.setFixedBaseMode(ui_state.lock_base);

    KinematicRosBridge ros_bridge(cfg.ros.enable);
    ros_bridge.initialize(argc, argv);

    std::string ikModeParam;
    std::string fullBodyBackendParam;
    int fullBodyIterationsParam = cfg.ik.full_body_iterations;
    if (ros_bridge.getParam("ik_mode", &ikModeParam)) {
        cfg.ik.mode = ikModeParam;
    }
    if (ros_bridge.getParam("ik_full_body_backend", &fullBodyBackendParam)) {
        cfg.ik.full_body_backend = fullBodyBackendParam;
    }
    if (ros_bridge.getParam("ik_full_body_iterations", &fullBodyIterationsParam)) {
        cfg.ik.full_body_iterations = std::max(1, fullBodyIterationsParam);
    }

    IkState ik_state;
    DebugPlaybackState playback_state;
    CollisionMonitorState collision_state;
    TrajectoryPlayer trajectory_player;
    CollisionMonitor collision_monitor;
    {
        ik_state.solve_mode = cfg.ik.mode;
        std::transform(ik_state.solve_mode.begin(), ik_state.solve_mode.end(), ik_state.solve_mode.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.solve_mode != "single_chain" && ik_state.solve_mode != "full_body") {
            ik_state.solve_mode = "single_chain";
        }
        ik_state.full_body_backend = cfg.ik.full_body_backend;
        std::transform(ik_state.full_body_backend.begin(), ik_state.full_body_backend.end(), ik_state.full_body_backend.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.full_body_backend != "flex_ik" && ik_state.full_body_backend != "wbc_chain_ik") {
            ik_state.full_body_backend = "flex_ik";
        }
        ik_state.full_body_iterations = cfg.ik.full_body_iterations;
        ik_state.solver.setFullBodyBackend(ik_state.full_body_backend);
        ik_state.solver.initialize(urdf_path, cfg.ik.chains);
        ik_state.chains.clear();
        for (int i = 0; i < ik_state.solver.chainCount(); ++i) {
            ik_state.chains.push_back(ik_state.solver.chainStatus(i));
        }
        ros_bridge.param<int>("ik_selected_chain", &ik_state.selected_chain, ik_state.selected_chain);
        if (!ik_state.chains.empty()) {
            ik_state.selected_chain = std::clamp(ik_state.selected_chain, 0, static_cast<int>(ik_state.chains.size()) - 1);
        }
        ik_state.marker_targets.resize(ik_state.chains.size());
    }

    ros_bridge.param<std::string>("ik_target_pose_topic", &ik_state.external_target_topic, ik_state.external_target_topic);
    ros_bridge.param<std::string>("ik_target_pose_frame", &ik_state.external_target_expected_frame,
                                  ik_state.external_target_expected_frame);
    ros_bridge.param<bool>("enable_external_ik_target", &ik_state.use_external_target, ik_state.use_external_target);
    ros_bridge.param<bool>("external_ik_target_position_only", &ik_state.external_target_position_only,
                           ik_state.external_target_position_only);
    if (!ros_bridge.enabled()) {
        ik_state.use_external_target = false;
    }

    ros_bridge.subscribeExternalTarget(ik_state.external_target_topic, 20, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            if (msg == nullptr) {
                return;
            }
            const std::string frame = msg->header.frame_id;
            if (!ik_state.external_target_expected_frame.empty() && frame != ik_state.external_target_expected_frame) {
                return;
            }
            ik_state.external_target_pos = glm::vec3(static_cast<float>(msg->pose.position.x), static_cast<float>(msg->pose.position.y),
                                                     static_cast<float>(msg->pose.position.z));
            ik_state.external_target_quat =
                glm::quat(static_cast<float>(msg->pose.orientation.w), static_cast<float>(msg->pose.orientation.x),
                          static_cast<float>(msg->pose.orientation.y), static_cast<float>(msg->pose.orientation.z));
            if (glm::length(ik_state.external_target_quat) > 1e-6f) {
                ik_state.external_target_quat = glm::normalize(ik_state.external_target_quat);
            } else {
                ik_state.external_target_quat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }
            ik_state.external_target_last_frame    = frame;
            ik_state.external_target_last_recv_sec = ros_bridge.nowSec();
            ik_state.external_target_received      = true;
            ik_state.external_target_dirty         = true;
        });

    double prev_x         = 0.0;
    double prev_y         = 0.0;
    bool first_mouse      = true;
    double last_frame_sec = glfwGetTime();

    auto ensureMarkerTargetInitialized = [&](int chain_index) -> bool {
        return EnsureMarkerTargetInitialized(&ik_state, &scene, chain_index);
    };

    auto loadActiveMarkerFromTarget = [&]() -> bool {
        return LoadActiveMarkerFromTarget(&ik_state, &scene);
    };

    auto saveActiveMarkerToTarget = [&]() {
        SaveActiveMarkerToTarget(&ik_state);
    };

    if (!ik_state.chains.empty()) {
        loadActiveMarkerFromTarget();
    }

    while (!glfwWindowShouldClose(window)) {
        ros_bridge.spinOnce();
        glfwPollEvents();
        double now_sec = glfwGetTime();
        double dt_sec  = std::max(0.0, now_sec - last_frame_sec);
        last_frame_sec = now_sec;

        double x = 0.0, y = 0.0;
        glfwGetCursorPos(window, &x, &y);
        if (first_mouse) {
            prev_x      = x;
            prev_y      = y;
            first_mouse = false;
        }
        double dx = x - prev_x;
        double dy = y - prev_y;
        prev_x    = x;
        prev_y    = y;

        int fb_w = 0, fb_h = 0;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        float panel_min      = 280.0f;
        float panel_max      = std::max(panel_min, static_cast<float>(fb_w) - 320.0f);
        ui_state.panel_width = std::clamp(ui_state.panel_width, panel_min, panel_max);
        int panel_w          = static_cast<int>(ui_state.panel_width);
        int viewport_w       = std::max(1, fb_w - panel_w);
        int viewport_h       = std::max(1, fb_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        bool mouse_in_viewport = (x >= 0.0 && x < static_cast<double>(viewport_w) && y >= 0.0 && y < static_cast<double>(viewport_h));
        const bool block_camera_input = ui_state.panel_resize_active;
        const bool imgui_capturing_mouse = ImGui::GetIO().WantCaptureMouse;
        if (mouse_in_viewport && !block_camera_input && !ik_state.dragging_marker && !ik_state.gizmo_was_using &&
            !ik_state.gizmo_was_over && !imgui_capturing_mouse) {
            bool left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            bool right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            bool shift  = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            if (left && !shift && !middle) {
                camera.rotate(static_cast<float>(dx), static_cast<float>(dy));
            } else if (middle || (shift && left)) {
                camera.pan(static_cast<float>(dx), static_cast<float>(dy));
            } else if (right) {
                camera.dolly(static_cast<float>(-dy));
            }
            if (std::fabs(g_scroll_delta) > 1e-6f) {
                camera.zoom(g_scroll_delta);
                g_scroll_delta = 0.0f;
            }
        }

        glViewport(0, 0, viewport_w, viewport_h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.90f, 0.92f, 0.96f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        trajectory_player.AdvanceAndApply(&playback_state, &scene, dt_sec);

        scene.setFixedBaseMode(ui_state.lock_base);
        scene.updateTransforms();
        CollisionMonitorResult collision_result = collision_monitor.Evaluate(collision_state, scene);
        collision_monitor.UpdateStateFromResult(collision_result, &collision_state);

        glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 view = camera.viewMatrix();
        glm::vec3 eye  = camera.eye();

        glUseProgram(mesh_shader);
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glm::vec3 light_pos = eye + glm::vec3(0.8f, 0.8f, 1.2f);
        glUniform3f(glGetUniformLocation(mesh_shader, "lightPos"), light_pos.x, light_pos.y, light_pos.z);
        glUniform3f(glGetUniformLocation(mesh_shader, "viewPos"), eye.x, eye.y, eye.z);
        scene.draw(mesh_shader);

        std::vector<KinematicLineVertex> axis_vertices;
        {
            float half = ui_state.grid_size;
            int count  = std::max(2, ui_state.grid_count);
            glm::vec3 grid_col(0.72f, 0.76f, 0.82f);
            for (int i = 0; i <= count; ++i) {
                float t = -half + 2.0f * half * (static_cast<float>(i) / static_cast<float>(count));
                axis_vertices.push_back({glm::vec3(-half, t, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(half, t, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(t, -half, 0.0f), grid_col});
                axis_vertices.push_back({glm::vec3(t, half, 0.0f), grid_col});
            }
        }
        if (ui_state.show_world_axes) {
            glm::vec3 o(0.0f);
            float l = ui_state.world_axis_length;
            axis_vertices.push_back({o, glm::vec3(1, 0, 0)});
            axis_vertices.push_back({o + glm::vec3(l, 0, 0), glm::vec3(1, 0, 0)});
            axis_vertices.push_back({o, glm::vec3(0, 1, 0)});
            axis_vertices.push_back({o + glm::vec3(0, l, 0), glm::vec3(0, 1, 0)});
            axis_vertices.push_back({o, glm::vec3(0, 0, 1)});
            axis_vertices.push_back({o + glm::vec3(0, 0, l), glm::vec3(0, 0, 1)});
        }
        if (ui_state.show_axes) {
            auto axes = scene.getJointAxisInfos(ui_state.show_revolute_only);
            for (const auto& a : axes) {
                if (!ui_state.show_non_revolute && !a.revolute) {
                    continue;
                }
                glm::vec3 c  = a.revolute ? glm::vec3(1.0f, 0.82f, 0.1f) : glm::vec3(0.45f, 0.8f, 1.0f);
                glm::vec3 p0 = a.world_origin - 0.5f * ui_state.axis_length * a.world_axis;
                glm::vec3 p1 = a.world_origin + 0.5f * ui_state.axis_length * a.world_axis;
                axis_vertices.push_back({p0, c});
                axis_vertices.push_back({p1, c});
            }
        }
        for (int i = 0; i < static_cast<int>(ik_state.marker_targets.size()); ++i) {
            if (!ik_state.marker_targets[i].initialized && !ensureMarkerTargetInitialized(i)) {
                continue;
            }
            const auto& target = ik_state.marker_targets[i];
            appendMarkerAxes(&axis_vertices, target.pos, target.rpy_deg, 0.12f, i == ik_state.selected_chain);
        }
        // Old custom marker is removed; ImGuizmo provides the interactive manipulator.

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            if (ik_state.selected_chain < 0 || ik_state.selected_chain >= static_cast<int>(ik_state.chains.size())) {
                ik_state.last_status = "IK失败：未选择链";
                return false;
            }
            const auto& chain_status = ik_state.chains[ik_state.selected_chain];
            if (ik_state.solve_mode == "single_chain" && !chain_status.ready) {
                ik_state.last_status = "IK失败：链未就绪";
                return false;
            }

            if (force_orientation_lock || ik_state.lock_orientation) {
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    ik_state.marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                    ik_state.marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                    ik_state.marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
                }
            }
            if (ik_state.solve_mode == "full_body" && fast_mode && prefer_position_only_target && !ik_state.lock_orientation) {
                // Realtime full-body drag uses position-priority target for robustness.
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    ik_state.marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                    ik_state.marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                    ik_state.marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
                }
            }
            saveActiveMarkerToTarget();

            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::mat4 active_target_world = markerWorldMatrix(marker_pos, marker_rpy_deg);

            if (ik_state.solve_mode == "full_body") {
                std::vector<glm::mat4> targets_world(static_cast<size_t>(ik_state.chains.size()), glm::mat4(1.0f));
                for (int i = 0; i < static_cast<int>(ik_state.chains.size()); ++i) {
                    if (i == ik_state.selected_chain) {
                        targets_world[static_cast<size_t>(i)] = active_target_world;
                        continue;
                    }

                    glm::vec3 tip_pos(0.0f);
                    glm::vec3 tip_rpy(0.0f);
                    if (ik_state.solver.fetchTipWorldPose(scene, i, &tip_pos, &tip_rpy)) {
                        const glm::vec3 tip_rpy_deg(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
                        targets_world[static_cast<size_t>(i)] = markerWorldMatrix(tip_pos, tip_rpy_deg);
                    } else if (ensureMarkerTargetInitialized(i)) {
                        const auto& target                    = ik_state.marker_targets[i];
                        targets_world[static_cast<size_t>(i)] = markerWorldMatrix(target.pos, target.rpy_deg);
                    } else {
                        targets_world[static_cast<size_t>(i)] = active_target_world;
                    }
                }

                IkSolveStats stats;
                const int solve_iters = fast_mode ? 1 : std::max(1, ik_state.full_body_iterations);
                if (ik_state.solver.solveFullBody(&scene, targets_world, solve_iters, ik_state.selected_chain, fast_mode,
                                                  prefer_position_only_target, &stats, &ik_state.last_status)) {
                    return true;
                }
                return false;
            }

            return ik_state.solver.solveSingleChain(&scene, ik_state.selected_chain, active_target_world, &ik_state.last_status);
        };

        auto refineActiveChainToMarker = [&]() -> bool {
            if (ik_state.selected_chain < 0 || ik_state.selected_chain >= static_cast<int>(ik_state.chains.size())) {
                return false;
            }
            const auto& chain_status = ik_state.chains[ik_state.selected_chain];
            if (!chain_status.ready) {
                return false;
            }
            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::mat4 target_world = markerWorldMatrix(marker_pos, marker_rpy_deg);
            std::string refine_status;
            const bool refined = ik_state.solver.solveSingleChain(&scene, ik_state.selected_chain, target_world, &refine_status);
            if (refined) {
                if (ik_state.last_status.empty()) {
                    ik_state.last_status = "末端精修完成(single_chain)";
                } else {
                    ik_state.last_status += " | 末端精修完成(single_chain)";
                }
                return true;
            }
            if (!refine_status.empty()) {
                if (ik_state.last_status.empty()) {
                    ik_state.last_status = "末端精修失败: " + refine_status;
                } else {
                    ik_state.last_status += " | 末端精修失败: " + refine_status;
                }
            }
            return false;
        };

        auto activeChainPositionErrorMmToMarker = [&]() -> float {
            if (ik_state.selected_chain < 0 || ik_state.selected_chain >= static_cast<int>(ik_state.chains.size())) {
                return 0.0f;
            }
            glm::vec3 tip_pos(0.0f);
            glm::vec3 tip_rpy(0.0f);
            if (!ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                return 0.0f;
            }
            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            return glm::length(marker_pos - tip_pos) * 1000.0f;
        };

        // External RViz interactive marker target -> local marker -> IK.
        if (ik_state.use_external_target && ik_state.external_target_dirty && !ik_state.dragging_marker) {
            const glm::vec3 prev_marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 prev_marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::quat prev_marker_quat = glm::normalize(glm::quat_cast(markerWorldMatrix(prev_marker_pos, prev_marker_rpy_deg)));
            const glm::quat target_quat      = glm::normalize(ik_state.external_target_quat);
            float external_rotation_delta    = glm::abs(glm::angle(glm::inverse(prev_marker_quat) * target_quat));
            if (external_rotation_delta > glm::pi<float>()) {
                external_rotation_delta = glm::two_pi<float>() - external_rotation_delta;
            }

            ik_state.marker_pos[0] = ik_state.external_target_pos.x;
            ik_state.marker_pos[1] = ik_state.external_target_pos.y;
            ik_state.marker_pos[2] = ik_state.external_target_pos.z;
            if (!ik_state.lock_orientation) {
                glm::vec3 rpy              = glm::eulerAngles(ik_state.external_target_quat);
                ik_state.marker_rpy_deg[0] = glm::degrees(rpy.x);
                ik_state.marker_rpy_deg[1] = glm::degrees(rpy.y);
                ik_state.marker_rpy_deg[2] = glm::degrees(rpy.z);
            }
            saveActiveMarkerToTarget();
            const bool external_position_only =
                ik_state.external_target_position_only && external_rotation_delta < glm::radians(0.5f);
            applyIkForActiveChain(false, false, external_position_only);
            ik_state.external_target_dirty = false;
        }

        // RViz-like manipulator via ImGuizmo
        // Draw gizmo directly on the foreground drawlist of the 3D viewport area.
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::SetGizmoSizeClipSpace(ik_state.gizmo_size_clip_space);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(viewport_w), static_cast<float>(viewport_h));

        if (ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size())) {
            if (!ik_state.marker_initialized) {
                loadActiveMarkerFromTarget();
            }

            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::mat4 marker_world_before = markerWorldMatrix(marker_pos, marker_rpy_deg);
            glm::mat4 gizmo_world               = marker_world_before;
            ImGuizmo::OPERATION op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (ik_state.gizmo_operation == 0)
                op = ImGuizmo::TRANSLATE;
            if (ik_state.gizmo_operation == 1)
                op = ImGuizmo::ROTATE;
            ImGuizmo::MODE mode = ik_state.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;

            float snap_values[3] = {0.0f, 0.0f, 0.0f};
            float* snap_ptr      = nullptr;
            if (op == ImGuizmo::TRANSLATE && ik_state.translate_snap_enabled) {
                snap_values[0] = ik_state.translate_snap_step_m;
                snap_values[1] = ik_state.translate_snap_step_m;
                snap_values[2] = ik_state.translate_snap_step_m;
                snap_ptr       = snap_values;
            } else if (op == ImGuizmo::ROTATE && ik_state.rotate_snap_enabled) {
                snap_values[0] = ik_state.rotate_snap_step_deg;
                snap_values[1] = ik_state.rotate_snap_step_deg;
                snap_values[2] = ik_state.rotate_snap_step_deg;
                snap_ptr       = snap_values;
            }

            glm::mat4 gizmo_delta(1.0f);
            bool manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, mode, glm::value_ptr(gizmo_world),
                                                    glm::value_ptr(gizmo_delta), snap_ptr);
            bool gizmo_using = ImGuizmo::IsUsing();
            bool gizmo_over  = ImGuizmo::IsOver();
            bool current_drag_position_only = (op == ImGuizmo::TRANSLATE);
            if (manipulated || gizmo_using) {
                ik_state.dragging_marker          = true;
                ik_state.gizmo_drag_interacted    = true;

                const glm::vec3 raw_pos = glm::vec3(gizmo_world[3]);
                const glm::quat raw_q   = glm::quat_cast(gizmo_world);
                const glm::vec3 raw_rpy_deg(glm::degrees(glm::eulerAngles(raw_q)));

                const glm::vec3 delta_pos = raw_pos - marker_pos;
                glm::vec3 scaled_delta_pos(delta_pos.x * ik_state.translate_channel_gain[0],
                                           delta_pos.y * ik_state.translate_channel_gain[1],
                                           delta_pos.z * ik_state.translate_channel_gain[2]);

                const glm::vec3 raw_delta_rpy_deg(wrapDeltaDeg(raw_rpy_deg.x - marker_rpy_deg.x),
                                                  wrapDeltaDeg(raw_rpy_deg.y - marker_rpy_deg.y),
                                                  wrapDeltaDeg(raw_rpy_deg.z - marker_rpy_deg.z));
                glm::vec3 scaled_delta_rpy_deg(raw_delta_rpy_deg.x * ik_state.rotate_channel_gain[0],
                                               raw_delta_rpy_deg.y * ik_state.rotate_channel_gain[1],
                                               raw_delta_rpy_deg.z * ik_state.rotate_channel_gain[2]);

                const glm::vec3 updated_pos = marker_pos + scaled_delta_pos;
                const glm::vec3 updated_rpy_deg(marker_rpy_deg.x + scaled_delta_rpy_deg.x, marker_rpy_deg.y + scaled_delta_rpy_deg.y,
                                                marker_rpy_deg.z + scaled_delta_rpy_deg.z);

                current_drag_position_only = glm::length(scaled_delta_pos) > 1e-6f &&
                                             glm::length(glm::radians(scaled_delta_rpy_deg)) < glm::radians(0.25f);
                if (op == ImGuizmo::TRANSLATE) {
                    current_drag_position_only = true;
                } else if (op == ImGuizmo::ROTATE) {
                    current_drag_position_only = false;
                }
                ik_state.gizmo_drag_position_only = current_drag_position_only;

                ik_state.marker_pos[0] = updated_pos.x;
                ik_state.marker_pos[1] = updated_pos.y;
                ik_state.marker_pos[2] = updated_pos.z;
                if (!ik_state.lock_orientation) {
                    ik_state.marker_rpy_deg[0] = updated_rpy_deg.x;
                    ik_state.marker_rpy_deg[1] = updated_rpy_deg.y;
                    ik_state.marker_rpy_deg[2] = updated_rpy_deg.z;
                }
                saveActiveMarkerToTarget();
                ik_state.gizmo_pose_dirty = true;
            } else {
                ik_state.dragging_marker = false;
            }

            // Realtime IK updates while dragging (RViz-like immediate feedback), throttled by frequency.
            if (gizmo_using && ik_state.gizmo_pose_dirty && ik_state.realtime_ik_during_drag) {
                float effective_hz = std::max(1.0f, ik_state.realtime_ik_hz);
                if (ik_state.solve_mode == "full_body") {
                    effective_hz = std::min(effective_hz, current_drag_position_only ? 12.0f : 4.0f);
                }
                const double interval_sec = 1.0 / static_cast<double>(effective_hz);
                if (ik_state.last_realtime_ik_apply_sec < 0.0 || (now_sec - ik_state.last_realtime_ik_apply_sec) >= interval_sec) {
                    applyIkForActiveChain(false, true, current_drag_position_only);
                    ik_state.last_realtime_ik_apply_sec = now_sec;
                    ik_state.gizmo_pose_dirty           = false;
                }
            }

            // Sync IK only when drag ends (mouse release / gizmo released)
            if (!gizmo_using && ik_state.gizmo_was_using && ik_state.gizmo_drag_interacted) {
                applyIkForActiveChain(false, false, ik_state.gizmo_drag_position_only);
                const bool drag_had_rotation = !ik_state.gizmo_drag_position_only;
                const bool should_refine_with_single_chain =
                    ik_state.solve_mode == "full_body" && ik_state.refine_single_chain_on_drag_end &&
                    (!ik_state.refine_only_when_rotation || drag_had_rotation);
                if (should_refine_with_single_chain) {
                    // Strongly close tip-to-marker gap after full-body drag end.
                    for (int pass = 0; pass < 3; ++pass) {
                        const float err_mm = activeChainPositionErrorMmToMarker();
                        if (err_mm <= 1.0f) {
                            break;
                        }
                        if (!refineActiveChainToMarker()) {
                            break;
                        }
                    }
                }
                ik_state.gizmo_pose_dirty         = false;
                ik_state.gizmo_drag_interacted    = false;
                ik_state.gizmo_drag_position_only = true;
            }
            ik_state.gizmo_was_using = gizmo_using;
            ik_state.gizmo_was_over  = gizmo_over;
        } else {
            ik_state.gizmo_was_using = false;
            ik_state.gizmo_was_over  = false;
        }

        // Marker hover/pick in viewport (screen-space)
        if (false && ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size()) &&
            !ImGui::GetIO().WantCaptureMouse) {
            glm::vec3 m0(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            bool okc                = false;
            glm::vec2 sc            = worldToScreen(m0, view, proj, viewport_w, viewport_h, &okc);
            float dist_px           = okc ? glm::length(glm::vec2(static_cast<float>(x), static_cast<float>(y)) - sc) : 1e9f;
            bool in_view            = x <= viewport_w;
            bool left_now           = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            ik_state.marker_hovered = in_view && okc && dist_px < 36.0f;
            if (ik_state.marker_hovered) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            if (left_now && !ik_state.left_mouse_prev && ik_state.marker_hovered) {
                ik_state.dragging_marker = true;
                ik_state.drag_prev_x     = static_cast<float>(x);
                ik_state.drag_prev_y     = static_cast<float>(y);
                ik_state.drag_prev_valid = true;
            }
            if (!left_now) {
                ik_state.dragging_marker = false;
                ik_state.drag_prev_valid = false;
            }
            ik_state.left_mouse_prev = left_now;

            if (ik_state.dragging_marker) {
                glm::vec3 axis(1.0f, 0.0f, 0.0f);
                if (ik_state.drag_mode == 2 || ik_state.drag_mode == 5)
                    axis = glm::vec3(0.0f, 1.0f, 0.0f);
                if (ik_state.drag_mode == 3 || ik_state.drag_mode == 6)
                    axis = glm::vec3(0.0f, 0.0f, 1.0f);
                if (ik_state.drag_mode == 0) {
                    glm::vec3 eye_now = camera.eye();
                    glm::vec3 forward = glm::normalize(camera.target - eye_now);
                    glm::vec3 world_up(0.0f, 0.0f, 1.0f);
                    glm::vec3 right = glm::cross(forward, world_up);
                    if (glm::length(right) < 1e-6f)
                        right = glm::vec3(1.0f, 0.0f, 0.0f);
                    else
                        right = glm::normalize(right);
                    glm::vec3 up    = glm::normalize(glm::cross(right, forward));
                    float scale     = std::max(0.00008f, camera.distance * 0.00018f) * ik_state.translate_sensitivity;
                    glm::vec3 delta = right * static_cast<float>(dx) * scale - up * static_cast<float>(dy) * scale;
                    ik_state.marker_pos[0] += delta.x;
                    ik_state.marker_pos[1] += delta.y;
                    ik_state.marker_pos[2] += delta.z;
                } else {
                    bool oka = false, okb = false;
                    glm::vec2 sa = worldToScreen(m0, view, proj, viewport_w, viewport_h, &oka);
                    glm::vec2 sb = worldToScreen(m0 + axis * 0.16f, view, proj, viewport_w, viewport_h, &okb);
                    if (oka && okb) {
                        glm::vec2 axis_px = sb - sa;
                        float axis_px_len = glm::length(axis_px);
                        if (axis_px_len > 1e-4f) {
                            glm::vec2 mouse_delta(static_cast<float>(dx), static_cast<float>(dy));
                            float delta_along_px = glm::dot(mouse_delta, axis_px / axis_px_len);
                            if (ik_state.drag_mode >= 1 && ik_state.drag_mode <= 3) {
                                float scale = std::max(0.00008f, camera.distance * 0.00018f) * ik_state.translate_sensitivity;
                                float d     = delta_along_px * scale;
                                ik_state.marker_pos[0] += axis.x * d;
                                ik_state.marker_pos[1] += axis.y * d;
                                ik_state.marker_pos[2] += axis.z * d;
                            } else {
                                int ridx = ik_state.drag_mode - 4;
                                if (ridx >= 0 && ridx < 3) {
                                    glm::vec2 center = sa;
                                    glm::vec2 v0(ik_state.drag_prev_x - center.x, ik_state.drag_prev_y - center.y);
                                    glm::vec2 v1(static_cast<float>(x) - center.x, static_cast<float>(y) - center.y);
                                    float l0 = glm::length(v0);
                                    float l1 = glm::length(v1);
                                    if (ik_state.drag_prev_valid && l0 > 1.0f && l1 > 1.0f) {
                                        v0 /= l0;
                                        v1 /= l1;
                                        float c   = glm::clamp(glm::dot(v0, v1), -1.0f, 1.0f);
                                        float ang = std::acos(c);
                                        float sgn = (v0.x * v1.y - v0.y * v1.x) >= 0.0f ? 1.0f : -1.0f;
                                        float deg = glm::degrees(ang) * sgn * ik_state.rotate_sensitivity;
                                        ik_state.marker_rpy_deg[ridx] += deg;
                                    }
                                }
                            }
                        }
                    }
                }
                ik_state.drag_prev_x     = static_cast<float>(x);
                ik_state.drag_prev_y     = static_cast<float>(y);
                ik_state.drag_prev_valid = true;
                saveActiveMarkerToTarget();
                applyIkForActiveChain(false, false, false);
            }
        }
        if (collision_state.enable && collision_state.show_closest_pair_line && collision_state.has_valid_distance) {
            glm::vec3 line_color(0.20f, 0.95f, 0.20f);
            if (collision_state.nearest_surface_distance_m <= collision_state.danger_distance_m) {
                line_color = glm::vec3(1.0f, 0.25f, 0.25f);
            } else if (collision_state.nearest_surface_distance_m <= collision_state.warning_distance_m) {
                line_color = glm::vec3(1.0f, 0.75f, 0.25f);
            }
            axis_vertices.push_back({collision_state.nearest_point_a, line_color});
            axis_vertices.push_back({collision_state.nearest_point_b, line_color});
        }

        line_renderer.draw(line_shader, axis_vertices, view, proj, ui_state.axis_line_width);

        glViewport(viewport_w, 0, panel_w, fb_h);
        glDisable(GL_DEPTH_TEST);
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(viewport_w), 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(panel_w), static_cast<float>(fb_h)), ImGuiCond_Always);
        ImGui::Begin("机器人运动学调试", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

        {
            ImGuiIO& io            = ImGui::GetIO();
            const float grip_width = 8.0f;
            ImVec2 window_pos      = ImGui::GetWindowPos();
            ImVec2 window_size     = ImGui::GetWindowSize();
            ImVec2 grip_min(window_pos.x, window_pos.y);
            ImVec2 grip_max(window_pos.x + grip_width, window_pos.y + window_size.y);
            ImGui::SetCursorScreenPos(grip_min);
            ImGui::InvisibleButton("##sidebar_resize_grip_kin", ImVec2(grip_width, window_size.y));
            bool grip_hovered            = ImGui::IsItemHovered();
            bool grip_active             = ImGui::IsItemActive();
            ui_state.panel_resize_active = grip_active;
            if (grip_hovered || grip_active) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            }
            if (grip_active) {
                ui_state.panel_width = std::clamp(ui_state.panel_width - io.MouseDelta.x, panel_min, panel_max);
            }
            ImU32 grip_color =
                grip_active ? IM_COL32(120, 200, 255, 220) : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
            ImGui::GetWindowDrawList()->AddRectFilled(grip_min, grip_max, grip_color, 2.0f);
            ImGui::SetCursorPosY(8.0f);
        }

        ImGui::TextWrapped("URDF: %s", urdf_path.c_str());
        ImGui::TextDisabled("视角：左键旋转，中键/Shift+左键平移，右键拖动缩放，滚轮缩放");
        ImGui::Separator();
        const char* sidebar_pages[] = {"场景", "IK", "回放", "安全", "关节", "TF"};
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::Combo("子页", &ui_state.sidebar_page, sidebar_pages, IM_ARRAYSIZE(sidebar_pages));
        ImGui::Separator();

        if (ui_state.sidebar_page == 0) {
            RenderScenePanel(&ui_state);
        }

        auto joints = scene.getJointInfos();
        if (ui_state.sidebar_page == 4) {
            RenderJointPanel(&ui_state, &scene, joints);
        }

        if (ui_state.sidebar_page == 1) {
            ImGui::Separator();
            ImGui::TextUnformatted("末端 Marker IK（TRAC-IK）");
            if (!ik_state.chains.empty()) {
            std::vector<const char*> chain_labels;
            chain_labels.reserve(ik_state.chains.size());
            for (const auto& c : ik_state.chains) {
                chain_labels.push_back(c.config.label.c_str());
            }
            if (ImGui::Combo("控制链", &ik_state.selected_chain, chain_labels.data(), static_cast<int>(chain_labels.size()))) {
                ik_state.marker_initialized = false;
                loadActiveMarkerFromTarget();
            }
            int solve_mode_index           = (ik_state.solve_mode == "full_body") ? 1 : 0;
            const char* solve_mode_items[] = {"single_chain（单链）", "full_body（全身）"};
            if (ImGui::Combo("求解模式", &solve_mode_index, solve_mode_items, IM_ARRAYSIZE(solve_mode_items))) {
                ik_state.solve_mode  = (solve_mode_index == 1) ? "full_body" : "single_chain";
                ik_state.last_status = "已切换IK求解模式";
            }
            if (ik_state.solve_mode == "full_body") {
                int backend_index           = (ik_state.full_body_backend == "wbc_chain_ik") ? 1 : 0;
                const char* backend_items[] = {"flex_ik", "wbc_chain_ik"};
                if (ImGui::Combo("全身IK后端", &backend_index, backend_items, IM_ARRAYSIZE(backend_items))) {
                    ik_state.full_body_backend = (backend_index == 1) ? "wbc_chain_ik" : "flex_ik";
                    ik_state.solver.setFullBodyBackend(ik_state.full_body_backend);
                    ik_state.last_status = std::string("已切换全身IK后端: ") + ik_state.full_body_backend;
                }
                ImGui::DragInt("全身迭代次数", &ik_state.full_body_iterations, 1.0f, 1, 30);
                ik_state.full_body_iterations = std::max(1, ik_state.full_body_iterations);
                ImGui::TextDisabled("彩色目标轴: 亮色=当前控制链，淡色=其他链");
            }
            ImGui::Checkbox("使用RViz目标位姿", &ik_state.use_external_target);
            ImGui::TextDisabled("外部位姿topic: %s", ik_state.external_target_topic.c_str());
            if (!ik_state.external_target_expected_frame.empty()) {
                ImGui::TextDisabled("外部位姿frame过滤: %s", ik_state.external_target_expected_frame.c_str());
            } else {
                ImGui::TextDisabled("外部位姿frame过滤: <未设置>");
            }
            if (!ros_bridge.enabled()) {
                ImGui::TextDisabled("外部位姿: ROS 未启用");
            } else if (ik_state.external_target_received) {
                double now_ros_sec = ros_bridge.nowSec();
                double age_sec     = std::max(0.0, now_ros_sec - ik_state.external_target_last_recv_sec);
                ImGui::Text("外部位姿: 已接收, frame=%s, %.2fs前",
                            ik_state.external_target_last_frame.empty() ? "<empty>" : ik_state.external_target_last_frame.c_str(), age_sec);
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 1.0f), "外部位姿: 未接收到消息");
            }

            const auto& chain_status = ik_state.chains[ik_state.selected_chain];
            if (ik_state.solve_mode == "single_chain" && !chain_status.ready) {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "IK链不可用: %s", chain_status.error.c_str());
            } else {
                if (!ik_state.marker_initialized) {
                    loadActiveMarkerFromTarget();
                }
                ImGui::Checkbox("锁定末端姿态", &ik_state.lock_orientation);
                ImGui::TextUnformatted("Gizmo 模式");
                ImGui::RadioButton("平移", &ik_state.gizmo_operation, 0);
                ImGui::SameLine();
                ImGui::RadioButton("旋转", &ik_state.gizmo_operation, 1);
                ImGui::SameLine();
                ImGui::RadioButton("平移+旋转", &ik_state.gizmo_operation, 2);
                ImGui::TextUnformatted("坐标系");
                if (ImGui::RadioButton("WORLD", ik_state.gizmo_world_mode)) {
                    ik_state.gizmo_world_mode = true;
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("LOCAL", !ik_state.gizmo_world_mode)) {
                    ik_state.gizmo_world_mode = false;
                }
                ImGui::SliderFloat("Gizmo 尺寸", &ik_state.gizmo_size_clip_space, 0.10f, 0.40f, "%.2f");
                ImGui::Checkbox("拖动时实时IK", &ik_state.realtime_ik_during_drag);
                if (ik_state.realtime_ik_during_drag) {
                    ImGui::SliderFloat("实时IK频率(Hz)", &ik_state.realtime_ik_hz, 5.0f, 120.0f, "%.0f");
                    if (ik_state.solve_mode == "full_body") {
                        ImGui::TextDisabled("full_body 拖动时频率自动上限：平移12Hz，姿态4Hz");
                        ImGui::TextDisabled("full_body 平移拖动走位置优先，旋转拖动走姿态求解");
                    }
                }
                if (ik_state.solve_mode == "full_body") {
                    ImGui::Checkbox("松手后末端精修(single_chain)", &ik_state.refine_single_chain_on_drag_end);
                    if (ik_state.refine_single_chain_on_drag_end) {
                        ImGui::Checkbox("仅旋转拖动时触发精修", &ik_state.refine_only_when_rotation);
                    }
                }
                ImGui::Checkbox("平移吸附", &ik_state.translate_snap_enabled);
                if (ik_state.translate_snap_enabled) {
                    ImGui::DragFloat("平移步长(m)", &ik_state.translate_snap_step_m, 0.001f, 0.001f, 0.20f, "%.3f");
                }
                ImGui::Checkbox("旋转吸附", &ik_state.rotate_snap_enabled);
                if (ik_state.rotate_snap_enabled) {
                    ImGui::DragFloat("旋转步长(度)", &ik_state.rotate_snap_step_deg, 0.2f, 0.2f, 45.0f, "%.1f");
                }
                ImGui::TextUnformatted("平移通道增益");
                ImGui::SliderFloat("Tx", &ik_state.translate_channel_gain[0], 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat("Ty", &ik_state.translate_channel_gain[1], 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat("Tz", &ik_state.translate_channel_gain[2], 0.0f, 2.0f, "%.2f");
                ImGui::TextUnformatted("旋转通道增益");
                ImGui::SliderFloat("Rx", &ik_state.rotate_channel_gain[0], 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat("Ry", &ik_state.rotate_channel_gain[1], 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat("Rz", &ik_state.rotate_channel_gain[2], 0.0f, 2.0f, "%.2f");
                ImGui::TextDisabled("直接在3D视窗抓取 Gizmo 轴/圆环进行平移或旋转");
                bool marker_pos_edited = ImGui::DragFloat3("Marker 位置(m)", ik_state.marker_pos, 0.002f, -2.0f, 2.0f, "%.4f");
                bool marker_pos_commit = ImGui::IsItemDeactivatedAfterEdit();
                ImGui::BeginDisabled(ik_state.lock_orientation);
                bool marker_rot_edited = ImGui::DragFloat3("Marker 姿态RPY(度)", ik_state.marker_rpy_deg, 0.2f, -180.0f, 180.0f, "%.2f");
                bool marker_rot_commit = ImGui::IsItemDeactivatedAfterEdit();
                ImGui::EndDisabled();
                if (marker_pos_edited || marker_rot_edited) {
                    saveActiveMarkerToTarget();
                    if (ik_state.realtime_ik_during_drag) {
                        const bool position_only_target = marker_pos_edited && !marker_rot_edited;
                        applyIkForActiveChain(false, true, position_only_target);
                    }
                } else if (!ik_state.realtime_ik_during_drag && (marker_pos_commit || marker_rot_commit)) {
                    const bool position_only_target = marker_pos_commit && !marker_rot_commit;
                    applyIkForActiveChain(false, false, position_only_target);
                }
                if (ImGui::Button("从当前末端同步Marker")) {
                    glm::vec3 tip_pos(0.0f);
                    glm::vec3 tip_rpy(0.0f);
                    if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                        ik_state.marker_pos[0]      = tip_pos.x;
                        ik_state.marker_pos[1]      = tip_pos.y;
                        ik_state.marker_pos[2]      = tip_pos.z;
                        ik_state.marker_rpy_deg[0]  = glm::degrees(tip_rpy.x);
                        ik_state.marker_rpy_deg[1]  = glm::degrees(tip_rpy.y);
                        ik_state.marker_rpy_deg[2]  = glm::degrees(tip_rpy.z);
                        ik_state.marker_initialized = true;
                        saveActiveMarkerToTarget();
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("求解 IK 并应用")) {
                    applyIkForActiveChain(false, false, false);
                }
                if (!ik_state.last_status.empty()) {
                    ImGui::TextUnformatted(ik_state.last_status.c_str());
                }
                ImGui::TextDisabled("base=%s  tip=%s", chain_status.config.base_link.c_str(), chain_status.config.tip_link.c_str());

                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state.solver.fetchTipWorldPose(scene, ik_state.selected_chain, &tip_pos, &tip_rpy)) {
                    glm::vec3 marker_p(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
                    float pos_err_mm = glm::length(marker_p - tip_pos) * 1000.0f;
                    glm::vec3 tip_rpy_deg(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
                    glm::vec3 marker_rpy(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
                    glm::vec3 drpy = glm::abs(marker_rpy - tip_rpy_deg);
                    ImVec4 ce      = (pos_err_mm < 2.0f)
                                         ? ImVec4(0.6f, 0.95f, 0.6f, 1.0f)
                                         : ((pos_err_mm < 8.0f) ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f) : ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                    ImGui::TextColored(ce, "末端误差: 位置 %.2f mm, 姿态 %.2f/%.2f/%.2f deg", pos_err_mm, drpy.x, drpy.y, drpy.z);
                }
            }
        }
        }

        if (ui_state.sidebar_page == 2) {
            RenderPlaybackPanel(&playback_state, &trajectory_player, &scene, joints);
        }

        if (ui_state.sidebar_page == 3) {
            RenderSafetyPanel(&collision_state, collision_result);
        }

        if (ui_state.sidebar_page == 5) {
            RenderTfPanel(&ui_state, scene.getLinkTfInfos());
        }

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    glDeleteProgram(mesh_shader);
    glDeleteProgram(line_shader);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    ros_bridge.shutdown();
    return 0;
}
