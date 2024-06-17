#ifndef QUAD_INVERSE_KINEMATICS_HPP
#define QUAD_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Assuming model.nq is accessible or defined elsewhere.
extern struct Model {
    int nq;
} model;

void quad_inverse_kinematics(double* q_in, double* foot_locs_in, double* foot_q_out);

#endif // QUAD_INVERSE_KINEMATICS_HPP