#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/edges.h>
#include <Eigen/Core>
#include <iostream>

// compatible meshes refer to 2 meshes with same mesh connectivity
#include "arap_interpolation.h"
Eigen::MatrixXd V_start, V_end, V_t;
Eigen::MatrixXi F, E;
Eigen::MatrixXd A;
std::vector<Eigen::Matrix2d> A_transforms;
std::vector<Eigen::Matrix2d> R;
std::vector<Eigen::Matrix2d> S;
Eigen::VectorXd angles;

int frames = 50;
int timestamp = 1;
bool updating = false;
int current_mesh = 0;
int current_interpolation = 0; // 0: ARAP, 1: Linear
int interpolation_progress = 0;

void arap_updateMesh(igl::opengl::glfw::Viewer& viewer) {
    double t = static_cast<double>(timestamp) / frames;
    computeInterpolatedVertices(A, A_transforms, R, S, angles, t, V_start, V_end, F, V_t);
    viewer.data().clear();
    viewer.data().set_mesh(V_t, F);
    viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Cyan
    viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Black
    viewer.core().align_camera_center(V_t, F);
}

void linear_updateMesh(igl::opengl::glfw::Viewer& viewer) {
    double t = static_cast<double>(timestamp) / frames;
    V_t = (1 - t) * V_start + t * V_end;
    viewer.data().clear();
    viewer.data().set_mesh(V_t, F);
    viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Cyan
    viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Black
    viewer.core().align_camera_center(V_t, F);
}

void updateMesh(igl::opengl::glfw::Viewer& viewer) {
    if (current_mesh == 2) {
        if (current_interpolation == 0) {
            arap_updateMesh(viewer);
        } else {
            linear_updateMesh(viewer);
        }
    }
}

void displayMesh(igl::opengl::glfw::Viewer& viewer, int mesh_id) {
    viewer.data().clear();
    switch (mesh_id) {
        case 0:
            viewer.data().set_mesh(V_start, F);
            viewer.data().set_colors(Eigen::RowVector3d(1.0, 1.0, 0.0)); // Yellow
            viewer.data().set_edges(V_start, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Black
            viewer.core().align_camera_center(V_start, F);
            break;
        case 1:
            viewer.data().set_mesh(V_end, F);
            viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 0.0)); // Green
            viewer.data().set_edges(V_end, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Black
            viewer.core().align_camera_center(V_end, F);
            break;
        case 2:
            viewer.data().set_mesh(V_t, F);
            viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Cyan
            viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Black
            viewer.core().align_camera_center(V_t, F);
            break;
        default:
            break;
    }
}

int main(int argc, char* argv[]) {
    igl::opengl::glfw::Viewer viewer;

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    if (!igl::readOBJ("../data/man.obj", V_start, F)) {
        std::cerr << "Error: Could not open ../data/man.obj" << std::endl;
        return -1;
    }

    if (!igl::readOBJ("../data/man2.obj", V_end, F)) {
        std::cerr << "Error: Could not open ../data/man0.obj" << std::endl;
        return -1;
    }

    V_t = V_start;
    igl::edges(F, E);
    computeRotationsAndTransforms(V_start, V_end, F, A, A_transforms, R, S, angles);

    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        if (ImGui::CollapsingHeader("Mesh Display", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* mesh_items[] = { "Start Mesh", "End Mesh", "Interpolated Mesh" };
            if (ImGui::Combo("Current Mesh", &current_mesh, mesh_items, IM_ARRAYSIZE(mesh_items))) {
                timestamp = 0;
                interpolation_progress = 0;
                displayMesh(viewer, current_mesh);
            }
        }

        if (ImGui::CollapsingHeader("Interpolation Method", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* interpolation_items[] = { "ARAP", "Linear" };
            if (ImGui::Combo("Interpolation", &current_interpolation, interpolation_items, IM_ARRAYSIZE(interpolation_items))) {
                timestamp = 0;
                interpolation_progress = 0;
                V_t = V_start;
                updateMesh(viewer);
            }
        }

        // Only show interpolation controls when interpolated mesh is selected
        if (current_mesh == 2) {
            ImGui::Begin("Interpolation Controls", NULL, ImGuiWindowFlags_AlwaysAutoResize);

            if (ImGui::SliderInt("Frame", &timestamp, 0, frames)) {
                interpolation_progress = timestamp;
                if (current_interpolation == 0) {
                    double t = static_cast<double>(timestamp) / frames;
                    computeInterpolatedVertices(A, A_transforms, R, S, angles, t, V_start, V_end, F, V_t);
                } else {
                    double t = static_cast<double>(timestamp) / frames;
                    V_t = (1 - t) * V_start + t * V_end;
                }
                viewer.data().clear();
                viewer.data().set_mesh(V_t, F);
                viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0));
                viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0));
                viewer.core().align_camera_center(V_t, F);
            }

            if (ImGui::Button("Reset")) {
                timestamp = 0;
                interpolation_progress = 0;
                V_t = V_start;
                viewer.data().clear();
                viewer.data().set_mesh(V_t, F);
                viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0));
                viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0));
                viewer.core().align_camera_center(V_t, F);
            }

            ImGui::End();
        }
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) -> bool {
        return false;
    };

    displayMesh(viewer, current_mesh);
    viewer.launch();
    return 0;
}