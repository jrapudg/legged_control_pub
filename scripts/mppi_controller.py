#!/usr/bin/env python

import rospy
import numpy as np
from control.mppi_gait import MPPI
from unitree_legged_msgs.msg import MotorState, MotorCmd, GaitState, GoalState
from nav_msgs.msg import Odometry

# KP_HIP_GAIN = 50
# KD_HIP_GAIN = 5

# KP_THIGH_GAIN = 50
# KD_THIGH_GAIN = 5

# KP_CALF_GAIN = 50
# KD_CALF_GAIN = 5

KP_HIP_GAIN = 60
KD_HIP_GAIN = 3

KP_THIGH_GAIN = 60
KD_THIGH_GAIN = 3

KP_CALF_GAIN = 60
KD_CALF_GAIN = 3


class Controller:
    def __init__(self):
        self.joint_command_publishers = {}
        self.joint_states = {"RL_hip":None, "RR_hip":None, "FL_hip":None, "FR_hip":None, 
                             "RL_thigh":None, "RR_thigh":None, "FL_thigh":None, "FR_thigh":None, 
                             "RL_calf":None, "RR_calf":None, "FL_calf":None, "FR_calf":None}
        
        self.controls = {"RL_hip":None, "RR_hip":None, "FL_hip":None, "FR_hip":None, 
                         "RL_thigh":None, "RR_thigh":None, "FL_thigh":None, "FR_thigh":None, 
                         "RL_calf":None, "RR_calf":None, "FL_calf":None, "FR_calf":None}
        
        self.Kp_gains = {"RL_hip":KP_HIP_GAIN, "RR_hip":KP_HIP_GAIN, "FL_hip":KP_HIP_GAIN, "FR_hip":KP_HIP_GAIN, 
                         "RL_thigh":KP_THIGH_GAIN, "RR_thigh":KP_THIGH_GAIN, "FL_thigh":KP_THIGH_GAIN, "FR_thigh":KP_THIGH_GAIN, 
                         "RL_calf":KP_CALF_GAIN, "RR_calf":KP_CALF_GAIN, "FL_calf":KP_CALF_GAIN, "FR_calf":KP_CALF_GAIN}
        self.Kd_gains = {"RL_hip":KD_HIP_GAIN, "RR_hip":KD_HIP_GAIN, "FL_hip":KD_HIP_GAIN, "FR_hip":KD_HIP_GAIN, 
                         "RL_thigh":KD_THIGH_GAIN, "RR_thigh":KD_THIGH_GAIN, "FL_thigh":KD_THIGH_GAIN, "FR_thigh":KD_THIGH_GAIN, 
                         "RL_calf":KD_CALF_GAIN, "RR_calf":KD_CALF_GAIN, "FL_calf":KD_CALF_GAIN, "FR_calf":KD_CALF_GAIN}
        
        self.body_pos = [0, 0, 0]
        self.body_ori = [1, 0, 0, 0]
        self.body_vel = [0, 0, 0]
        self.body_ang_vel = [0, 0, 0]

        self.body_pos_goal = [0, 0, 0.2]
        self.body_ori_goal = [0.92388, 0, 0, 0.38268]

        self.joits_ref = None

    def joint_state_callback(self, data, joint_name):
        # Store the latest state data
        self.joint_states[joint_name] = data

    def body_state_callback(self, data):
        # Store the latest state data
        self.body_pos = data.body_pos
        self.body_ori = data.body_ori
        self.body_vel = data.body_vel
        self.body_ang_vel = data.body_ang_vel
    
    def odom_callback(self, data):
        # Store the latest state data
        #print("odom_callback")
        self.body_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
        self.body_vel = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

    def mocap_pos_callback(self, data):
        # Store the latest state data
        # print("odom_callback")
        self.body_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
    
    def odom_vel_callback(self, data):
        # Store the latest state data
        # print("odom_callback")
        self.body_vel = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

    def gait_callback(self, data):
        gait_data = data.gait
        # Store the latest state data
        self.joits_ref = np.array(gait_data.data).reshape(gait_data.layout.dim[0].size, gait_data.layout.dim[1].size)

    def goal_callback(self, data):
        self.body_pos_goal = data.body_pos
        self.body_ori_goal = data.body_ori

    def loop(self):
        rospy.init_node('controller_quadruped', anonymous=True)
        rate = rospy.Rate(50) 

        # List of joints to control
        joints = ["RL_hip", "RR_hip", "FL_hip", "FR_hip", 
                  "RL_thigh", "RR_thigh", "FL_thigh", "FR_thigh", 
                  "RL_calf", "RR_calf", "FL_calf", "FR_calf"]
        
        # odom_topic = "/ground_truth/state"
        # pos_topic = "/odom"
        # pos_topic = "/mocap_node/Go1_body/Odom/"
        pos_topic = "/ground_truth/state"
        vel_topic = "/odom"
        gait_topic = "/quadruped/gait"
        goal_topic = '/quadruped/goal'

        # Create subscribers and publishers for each joint
        for joint in joints:
            state_topic = f"/joint_controller_{joint}/state"
            command_topic = f"/joint_controller_{joint}/cmd"
            
            # Subscribe to joint state
            rospy.Subscriber(state_topic, MotorState, self.joint_state_callback, joint)

            # Publisher for joint commands
            pub = rospy.Publisher(command_topic, MotorCmd, queue_size=1)
            self.joint_command_publishers[joint] = pub
        
        rospy.Subscriber(pos_topic, Odometry, self.mocap_pos_callback)
        rospy.Subscriber(vel_topic, Odometry, self.odom_vel_callback)
        #rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        #rospy.Subscriber(body_topic, QuadrupedState, self.body_state_callback)
        #rospy.Subscriber(pos_topic, Odometry, self.mocap_pos_callback)
        #rospy.Subscriber(vel_topic, Odometry, self.odom_vel_callback)

        rospy.Subscriber(gait_topic, GaitState, self.gait_callback)
        rospy.Subscriber(goal_topic, GoalState, self.goal_callback)
        
        rospy.sleep(1)
        print("Pos: {}".format(self.body_pos))
        print("Ori: {}".format(self.body_ori))
        print("Lin: {}".format(self.body_vel))
        print("Ang: {}".format(self.body_ang_vel))

        # Set up a ROS rate to manage publishing speed
        mppi = MPPI()
        while not rospy.is_shutdown():
            print()
            # state = np.concatenate([self.body_pos, self.body_ori, 
            #                         [self.joint_states["FL_hip"].q, self.joint_states["FL_thigh"].q, self.joint_states["FL_calf"].q],
            #                         [self.joint_states["FR_hip"].q, self.joint_states["FR_thigh"].q, self.joint_states["FR_calf"].q],
            #                         [self.joint_states["RL_hip"].q, self.joint_states["RL_thigh"].q, self.joint_states["RL_calf"].q],
            #                         [self.joint_states["RR_hip"].q, self.joint_states["RR_thigh"].q, self.joint_states["RR_calf"].q],
            #                         self.body_vel, self.body_ang_vel,
            #                         [self.joint_states["FL_hip"].dq, self.joint_states["FL_thigh"].dq, self.joint_states["FL_calf"].dq],
            #                         [self.joint_states["FR_hip"].dq, self.joint_states["FR_thigh"].dq, self.joint_states["FR_calf"].dq],
            #                         [self.joint_states["RL_hip"].dq, self.joint_states["RL_thigh"].dq, self.joint_states["RL_calf"].dq],
            #                         [self.joint_states["RR_hip"].dq, self.joint_states["RR_thigh"].dq, self.joint_states["RR_calf"].dq]
            #                         ])

            state = np.concatenate([self.body_pos, self.body_ori, 
                                    [self.joint_states["FR_hip"].q, self.joint_states["FR_thigh"].q, self.joint_states["FR_calf"].q],
                                    [self.joint_states["FL_hip"].q, self.joint_states["FL_thigh"].q, self.joint_states["FL_calf"].q],
                                    [self.joint_states["RR_hip"].q, self.joint_states["RR_thigh"].q, self.joint_states["RR_calf"].q],
                                    [self.joint_states["RL_hip"].q, self.joint_states["RL_thigh"].q, self.joint_states["RL_calf"].q],
                                    self.body_vel, self.body_ang_vel,
                                    [self.joint_states["FR_hip"].dq, self.joint_states["FR_thigh"].dq, self.joint_states["FR_calf"].dq],
                                    [self.joint_states["FL_hip"].dq, self.joint_states["FL_thigh"].dq, self.joint_states["FL_calf"].dq],
                                    [self.joint_states["RR_hip"].dq, self.joint_states["RR_thigh"].dq, self.joint_states["RR_calf"].dq],
                                    [self.joint_states["RL_hip"].dq, self.joint_states["RL_thigh"].dq, self.joint_states["RL_calf"].dq]
                                    ])
            
            mppi.joints_ref = self.joits_ref[:, :mppi.horizon]
            mppi.body_ref = np.concatenate((self.body_pos_goal, self.body_ori_goal, np.zeros(6)))
            #print( mppi.body_ref)
            control_effort = mppi.update(state)
            #control_effort = mppi.update(state, update_ref=True)

            self.controls["FR_hip"] = control_effort[0]
            self.controls["FR_thigh"] = control_effort[1]
            self.controls["FR_calf"] = control_effort[2]

            self.controls["FL_hip"] = control_effort[3]
            self.controls["FL_thigh"] = control_effort[4]
            self.controls["FL_calf"] = control_effort[5]

            self.controls["RR_hip"] = control_effort[6]
            self.controls["RR_thigh"] = control_effort[7]
            self.controls["RR_calf"] = control_effort[8]

            self.controls["RL_hip"] = control_effort[9]
            self.controls["RL_thigh"] = control_effort[10]
            self.controls["RL_calf"] = control_effort[11]

            
            for joint_name, data in self.joint_states.items():
                # Control logic: apply a simple proportional controller                    
                # Create and publish the command message
                command_msg = MotorCmd()
                command_msg.mode = 0x0A  # Position control mode
                command_msg.q = self.controls[joint_name]
                command_msg.dq = 0 
                command_msg.tau = 0 #control_effort
                command_msg.Kp = self.Kp_gains[joint_name]  # Position gain
                command_msg.Kd = self.Kd_gains[joint_name]    # Damping gain

                if joint_name in self.joint_command_publishers:
                    self.joint_command_publishers[joint_name].publish(command_msg)
                    rospy.loginfo(f"Control command for {joint_name}: Position = {self.controls[joint_name]}")


            rate.sleep()  # Sleep to maintain the loop rate at 50 Hz

if __name__ == '__main__':
    controller = Controller()
    controller.loop()