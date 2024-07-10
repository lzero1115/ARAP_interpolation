//
// Created by lex on 7/10/24.
//
#include "arap_interpolation.h"
#include <igl/edge_flaps.h>
#include <igl/svd3x3.h>
#include <igl/polar_svd.h>


// 2d interpolation
void computeRotationsAndTransforms(
    const Eigen::MatrixXd& V_start,
    const Eigen::MatrixXd& V_end,
    const Eigen::MatrixXi& faces,
    Eigen::MatrixXd& A,
    vector<Eigen::Matrix2d>& A_transforms,
    vector<Eigen::Matrix2d>& R,
    vector<Eigen::Matrix2d>& S,
    Eigen::VectorXd& angles)
{
    int num_faces = faces.rows();
    int num_vertices = V_start.rows();

    A = Eigen::MatrixXd::Zero(4 * num_faces, 2 * num_vertices);
    A_transforms.resize(num_faces);
    R.resize(num_faces);
    S.resize(num_faces);
    angles.resize(num_faces);

    for(int i=0;i<num_faces;i++)
    {
        Eigen::Vector3i face = faces.row(i);

        Eigen::Matrix<double, 6, 6> P;
        P << V_start.row(face[0])[0],V_start.row(face[0])[1],1,0,0,0,
             0,0,0,V_start.row(face[0])[0],V_start.row(face[0])[1],1,
             V_start.row(face[1])[0],V_start.row(face[1])[1],1,0,0,0,
             0,0,0,V_start.row(face[1])[0],V_start.row(face[1])[1],1,
             V_start.row(face[2])[0],V_start.row(face[2])[1],1,0,0,0,
             0,0,0,V_start.row(face[2])[0],V_start.row(face[2])[1],1;

        Eigen::Matrix<double, 6, 1> Q;
        Q << V_end.row(face[0])[0],
             V_end.row(face[0])[1],
             V_end.row(face[1])[0],
             V_end.row(face[1])[1],
             V_end.row(face[2])[0],
             V_end.row(face[2])[1];

        // Solve for PX = Q
        Eigen::Matrix<double, 6, 1> X = P.colPivHouseholderQr().solve(Q); // (a,b,tx,c,d,ty)
        Eigen::Matrix2d A_transform;
        A_transform<< X(0),X(1),
                      X(3),X(4);
        Eigen::Matrix2d R_mat, T_mat, U, V;
        Eigen::Vector2d S_vec;

        igl::polar_svd(A_transform, true, R_mat, T_mat, U, S_vec, V); // A = RT
        angles[i] = atan2(R_mat(1, 0), R_mat(0, 0));

        A_transforms[i] = A_transform;
        R[i] = R_mat;
        S[i] = T_mat;
        Eigen::Matrix<double,6,6> P_inv = P.inverse();
        for(int v=0;v<3;v++)
        {
            A(i*4, 2*face(v)) = P_inv(0,2*v);
            A(i*4, 2*face(v)+1) = P_inv(0,2*v+1);
            A(i*4+1, 2*face(v)) = P_inv(1,2*v);
            A(i*4+1, 2*face(v)+1) = P_inv(1,2*v+1);
            A(i*4+2, 2*face(v)) = P_inv(3,2*v);
            A(i*4+2, 2*face(v)+1) = P_inv(3,2*v+1);
            A(i*4+3, 2*face(v)) = P_inv(4,2*v);
            A(i*4+3, 2*face(v)+1) = P_inv(4,2*v+1);
        }
    }
}

void computeInterpolatedVertices(
    const Eigen::MatrixXd& A,
    const vector<Eigen::Matrix2d>& A_transforms,
    const vector<Eigen::Matrix2d>& R,
    const vector<Eigen::Matrix2d>& S,
    const Eigen::VectorXd& angles,
    double t,
    const Eigen::MatrixXd& V_start,
    const Eigen::MatrixXd& V_end,
    const Eigen::MatrixXi& faces,
    Eigen::MatrixXd& V_t)
{
    int num_faces = faces.rows();
    int num_vertices = V_start.rows();
    Eigen::MatrixXd I = Eigen::Matrix2d::Identity();
    Eigen::VectorXd b = Eigen::VectorXd::Zero(4 * num_faces);

    Eigen::MatrixXd A_first = A.leftCols(2);
    Eigen::MatrixXd A_rest = A.rightCols(A.cols() - 2); // interpolate the 1st vertex

    Eigen::VectorXd inner = A_first.col(0) * ((1 - t) * V_start(0, 0) + t * V_end(0, 0)) +
                            A_first.col(1) * ((1 - t) * V_start(0, 1) + t * V_end(0, 1));

    for (int i = 0; i < num_faces; i++) {
        Eigen::Matrix2d R_t;
        R_t << cos(angles[i] * t), -sin(angles[i] * t),
               sin(angles[i] * t), cos(angles[i] * t);

        Eigen::Matrix2d A_t = R_t * ((1 - t) * I + t * S[i]);

        b(i * 4) = A_t(0, 0);
        b(i * 4 + 1) = A_t(0, 1);
        b(i * 4 + 2) = A_t(1, 0);
        b(i * 4 + 3) = A_t(1, 1);
    }

    b = b - inner;

    Eigen::VectorXd V = (A_rest.transpose() * A_rest).ldlt().solve(A_rest.transpose() * b);

    V_t(0, 0) = (1 - t) * V_start(0, 0) + t * V_end(0, 0);
    V_t(0, 1) = (1 - t) * V_start(0, 1) + t * V_end(0, 1);

    for (int i = 1; i < num_vertices; ++i) {
        V_t(i, 0) = V(2 * (i - 1));
        V_t(i, 1) = V(2 * (i - 1) + 1);
    }
}
