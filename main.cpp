#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/edges.h>
#include <Eigen/Core>
#include <iostream>

// same connection for 2 meshes
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
    if(current_mesh==2)
    {
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

        if (ImGui::CollapsingHeader("Interpolation Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Start/Pause") && current_mesh == 2) {
                updating = !updating;
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset") && current_mesh == 2) {
                updating = false;
                timestamp = 1;
                V_t = V_start;
                updateMesh(viewer);
            }
        }

        if (ImGui::CollapsingHeader("Mesh Display", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* mesh_items[] = { "Start Mesh", "End Mesh", "Interpolated Mesh" };
            if (ImGui::Combo("Current Mesh", &current_mesh, mesh_items, IM_ARRAYSIZE(mesh_items))) {
                updating = false;  // Stop updating when switching mesh
                displayMesh(viewer, current_mesh);
            }
        }

        if (ImGui::CollapsingHeader("Interpolation Method", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* interpolation_items[] = { "ARAP", "Linear" };
            if (ImGui::Combo("Current Interpolation", &current_interpolation, interpolation_items, IM_ARRAYSIZE(interpolation_items))) {
                updating = false;  // Stop updating when switching interpolation method
                timestamp = 1;
                V_t = V_start;
                updateMesh(viewer);
            }
        }
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) -> bool {
        if (updating && current_mesh == 2) {
            if (timestamp == (frames + 1)) {
                timestamp = 1;
            }
            updateMesh(viewer);
            timestamp++;
        }
        return false; // Returning false allows for the ImGui plugin to handle rendering
    };

    displayMesh(viewer, current_mesh);
    viewer.launch();
    return 0;
}




// bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers) {
//     if (key == '1')
//     {
//         // Clear should be called before drawing the mesh
//         updating = false;
//         viewer.data().clear();
//         // Draw_mesh creates or updates the vertices and faces of the displayed mesh.
//         // If a mesh is already displayed, draw_mesh returns an error if the given V and
//         // F have size different than the current ones
//         viewer.data().set_mesh(V_start, F);
//         viewer.data().set_colors(Eigen::RowVector3d(1.0, 1.0, 0.0)); // Set the mesh color to yellow
//         viewer.data().set_edges(V_start, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Set the edge color to black
//         viewer.core().align_camera_center(V_start,F);
//     }
//     else if (key == '2')
//     {
//         updating = false;
//         viewer.data().clear();
//         viewer.data().set_mesh(V_end, F);
//         viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 0.0)); // Set the mesh color to yellow
//         viewer.data().set_edges(V_end, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Set the edge color to black
//         viewer.core().align_camera_center(V_end,F);
//     }
//     else if (key == '3')
//     {
//         updating = true;
//         viewer.data().clear();
//         viewer.data().set_mesh(V_t, F);
//         viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Set the mesh color to yellow
//         viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Set the edge color to black
//         viewer.core().align_camera_center(V_t,F);
//     }
//     else if (key == 'r' || key == 'R')
//     {
//         updating = true;
//         timestamp = 1;
//         V_t = V_start;
//         viewer.data().clear();
//         viewer.data().set_mesh(V_t, F);
//         viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Set the mesh color to yellow
//         viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Set the edge color to black
//         viewer.core().align_camera_center(V_t,F);
//     }
//
//     return false;
// }
//
// bool mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier) {
//     if (updating && button == GLFW_MOUSE_BUTTON_LEFT) {
//         if (timestamp == (frames + 1)) {
//             timestamp = 1;
//         }
//
//         double t = static_cast<double>(timestamp) / frames;
//         // std::cout << t << std::endl;
//         //clock_t start_time = clock();
//
//         //V_t = (1 - t) * V_start + t * V_end; //linear interpolation
//         computeInterpolatedVertices(A,A_transforms,R,S,angles,t,V_start,V_end,F,V_t);
//
//         //clock_t elapsed_time = clock() - start_time;
//         //std::cout << "Time taken for Linear Interpolation: " << static_cast<double>(elapsed_time) / CLOCKS_PER_SEC << " seconds" << std::endl;
//         timestamp++;
//         viewer.data().clear();
//         viewer.data().set_mesh(V_t, F);
//         viewer.data().set_colors(Eigen::RowVector3d(0.0, 1.0, 1.0)); // Set the mesh color to yellow
//         viewer.data().set_edges(V_t, E, Eigen::RowVector3d(0.0, 0.0, 0.0)); // Set the edge color to black
//         viewer.core().align_camera_center(V_t,F);
//
//     }
//     return false;
// }