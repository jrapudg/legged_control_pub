#include "inverse_kinematics.hpp"
#include <iostream>
#include <iomanip> // For std::setprecision

// Example usage of the quad_inverse_kinematics function.
int main() {
    // Example inputs: Adjust these based on the expected robot configuration and foot locations.
    // Assuming model.nq is 7 for this example: 3 for position, 4 for orientation.
    double q_in[] = {0.0, -0.0, -0.27, 1.0, 0.0, 0.0, 0.0}; // Base position and orientation (quaternion)
    double foot_locs_in[] = {
        0.1881, -0.12675, -0.00519415,  // Foot 1
        0.1881, 0.12675, -0.00519415,   // Foot 2
        -0.1881, -0.12675, -0.00519415, // Foot 3
        -0.1881, 0.12675, -0.00519415   // Foot 4
    };

    // Output array to hold the joint angles for each leg. Assuming 3 joints per leg, 2 possible solutions per joint.
    double foot_q_out[24]; // 12 joints * 2 solutions

    // Call the function
    quad_inverse_kinematics(q_in, foot_locs_in, foot_q_out);

    // Output the results
    std::cout << "Inverse Kinematics Results:" << std::endl;

    std::cout << "[";
    for (int i = 0; i < 12; i += 1) {
        std::cout << std::setprecision(4) << foot_q_out[i] << " ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "[";
    for (int i = 12; i < 24; i += 1) {
        std::cout << std::setprecision(4) << foot_q_out[i] << " ";
    }
    std::cout << "]" << std::endl;
    return 0;
}