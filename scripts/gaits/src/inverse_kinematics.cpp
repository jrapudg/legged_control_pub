#include "inverse_kinematics.hpp"
#include <iostream>

void quad_inverse_kinematics(double* q_in, double* foot_locs_in, double* foot_q_out) {
        // Pull in arguments
        Eigen::Map<Eigen::VectorXd> q(q_in, 7);
        Eigen::Map<Eigen::VectorXd> foot_locs(foot_locs_in, 12);

        // Generate base frame from quaternion and translation
        Eigen::Quaternion<double> rot(q.segment<4>(3));
        // GeometryData::SE3 baseFrame(rot, q.segment<3>(0)); 
        Eigen::Matrix3d base_rot = rot.toRotationMatrix();
        Eigen::Vector3d base_pos = q.segment<3>(0);

        std::cout << "base_pos: [" << base_pos[0] << ", " << base_pos[1] << ", " << base_pos[2] << "]" << std::endl;
        // Pre-allocate global footFrame
        // GeometryData::SE3 footFrame(baseFrame.rotation(), Eigen::Vector3d::Zero());

        // Leg variables
        double h = 0.08; // Offset from hip to thigh
        double l1 = 0.213; // Thigh length
        double l2 = 0.213; // Calf length

        // Pre-allocate storage for result (should be 2 solutions in most cases)
        Eigen::Matrix<double, 12, 2> foot_q;

        std::vector<Eigen::Vector3d> hip_local_pos(4);
        hip_local_pos[0] << 0.1881, 0.04675, 0;
        hip_local_pos[1] << 0.1881, -0.04675, 0;
        hip_local_pos[2] << -0.1881, 0.04675, 0;
        hip_local_pos[3] << -0.1881, -0.04675, 0;

        // Perform IK for each foot
        for(int footInd = 0; footInd < 4; footInd++) {      
            // Set footFrame translation based on foot location
            // footFrame.translation(foot_locs.segment<3>(3*footInd));

            // Get global hip frame
            // const auto& hipFrame = baseFrame.act(hip_local_frames[footInd]);
            Eigen::Vector3d hip_trans = base_rot.transpose()*base_pos;
            std::cout << "base_pos_rot: [" << hip_trans[0] << ", " << hip_trans[1] << ", " << hip_trans[2] << "]" << std::endl;
            // Get foot to hip frame, and get position vector to foot in hip frame
            // const auto& footInHip = hipFrame.inverse().act(footFrame);
            // Eigen::Vector3d footPos = footInHip.translation();
            

            std::cout << "hip_local_pos: [" << hip_local_pos[footInd][0] << ", " << hip_local_pos[footInd][1] << ", " << hip_local_pos[footInd][2] << "]" << std::endl;
            hip_trans = -hip_local_pos[footInd] - base_rot.transpose()*base_pos;
            Eigen::Vector3d footPos = base_rot.transpose()*foot_locs.segment<3>(3*footInd) + hip_trans;
            
            std::cout << "hip_trans: [" << hip_trans[0] << ", " << hip_trans[1] << ", " << hip_trans[2] << "]" << std::endl;

            std::cout << "footPos: [" << footPos[0] << ", " << footPos[1] << ", " << footPos[2] << "]" << std::endl;

            // Reflect right feet to use same IK for all four feet
            if (footInd % 2 == 1) footPos[1] = -footPos[1];

            // Variables to make code cleaner
            double x = footPos[0];
            double y = footPos[1];
            double z = footPos[2];

            //-- Calculate hip angle --//
            std::cout << "x" << x << std::endl;
            std::cout << "y" << y << std::endl;
            std::cout << "z" << z << std::endl;
            // Calculate leg length if hip angle was 0
            double L_squared = pow(y, 2) + pow(z, 2) - pow(h, 2);
            double L = 0;
            if (L_squared > 1e-12) L = sqrt(L_squared); //Prevent numerical issues if L is close to 0
            std::cout << "L_squared: [" << L_squared << std::endl;
            std::cout << "L" << L << std::endl;
            // Solve linear system in cos(theta), sin(theta) relating leg vector
            // before and after rotation. There are two solutions corresponding to (h, L) and (h, -L)
            double cos_theta = (h*y - L*z) / (pow(L, 2) + pow(h, 2));
            double sin_theta = (L*y + h*z) / (pow(L, 2) + pow(h, 2));

            std::cout << "cos_theta: " << cos_theta << std::endl;
            std::cout << "sin_theta: " << sin_theta << std::endl;
            foot_q(footInd*3, 0) = atan2(sin_theta, cos_theta); // First solution

            std::cout << "footPos: [" << foot_q(0, 0) << ", " << foot_q(1, 0) << ", " << foot_q(2, 0) << "]" << std::endl;
            std::cout << "footPos: [" << foot_q(3, 0) << ", " << foot_q(4, 0) << ", " << foot_q(5, 0) << "]" << std::endl;
            cos_theta = (h*y + L*z) / (pow(L, 2) + pow(h, 2));
            sin_theta = (-L*y + h*z) / (pow(L, 2) + pow(h, 2));
            foot_q(footInd*3, 1) = atan2(sin_theta, cos_theta); // Second solution

            
            for (int j = 0; j < 2; j++) {
                for (int i = 0; i < 12; i++) {
                    std::cout << foot_q(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;

            //-- Calculate thigh and calf angle for each possible hip angle --//
            for (int j = 0; j < 2; j++) {
                // Undo hip rotation on z-axis to deal with planar xz calf-thigh relationship
                double z_rot = sin(-foot_q(footInd*3, j))*y + cos(-foot_q(footInd*3, j))*z;

                foot_q(footInd*3 + 2, j) = acos((pow(l1, 2) + pow(l2, 2) - 
                                                (pow(x, 2) + pow(z_rot, 2)))/(2*pow(l1, 2))) - M_PI;
                foot_q(footInd*3 + 1, j) = -foot_q(footInd*3 + 2, j) / 2 - atan2(z_rot, x) - M_PI/2;
            }

            // Flip hip angles for right feet
            if (footInd % 2 == 1) {
                foot_q(footInd*3, 0) = -foot_q(footInd*3, 0);
                foot_q(footInd*3, 1) = -foot_q(footInd*3, 1);
            }
        }

        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 2; j++) {
                foot_q_out[i + j*12] = foot_q(i, j);
            }
        }
        
    }