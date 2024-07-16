#!/usr/bin/env python

import rospy
import numpy as np
from unitree_legged_msgs.msg import MotorCmd
from nav_msgs.msg import Odometry

KP_HIP_GAIN = 50
KD_HIP_GAIN = 3

KP_THIGH_GAIN = 50
KD_THIGH_GAIN = 3

KP_CALF_GAIN = 50
KD_CALF_GAIN = 3

class Controller:
    def __init__(self):
        self.joint_command_publishers = {}
        # self.joints_ref = {"RL_hip":0.3, "RR_hip":-0.3, "FL_hip":0.3, "FR_hip":-0.3, 
        #                   "RL_thigh":0.9, "RR_thigh":0.9, "FL_thigh":0.9, "FR_thigh":0.9, 
        #                   "RL_calf":-1.8, "RR_calf":-1.8, "FL_calf":-1.8, "FR_calf":-1.8}
        # self.joints_ref = {"RL_hip":0, "RR_hip":0, "FL_hip":0, "FR_hip":0,
        #                    "RL_thigh":1.34, "RR_thigh":1.34, "FL_thigh":1.34, "FR_thigh":1.34,
        #                    "RL_calf":-2.83, "RR_calf":-2.83, "FL_calf":-2.83, "FR_calf":-2.83}

        # self.joints_ref = {"RL_hip":0.7, "RR_hip":-0.7, "FL_hip":0.7, "FR_hip":-0.7,
        #                    "RL_thigh":1.34, "RR_thigh":1.34, "FL_thigh":1.34, "FR_thigh":1.34,
        #                    "RL_calf":-2.83, "RR_calf":-2.83, "FL_calf":-2.83, "FR_calf":-2.83}

        # self.joints_ref = {"RL_hip":0.1, "RR_hip":-0.1, "FL_hip":0.1, "FR_hip":-0.1,
        #                    "RL_thigh":1.34, "RR_thigh":1.34, "FL_thigh":1.34, "FR_thigh":1.34,
        #                    "RL_calf":-2.83, "RR_calf":-2.83, "FL_calf":-2.83, "FR_calf":-2.83}

        self.joints_ref = {"RL_hip":0.1, "RR_hip":-0.1, "FL_hip":0.1, "FR_hip":-0.7,
                           "RL_thigh":1.34, "RR_thigh":1.34, "FL_thigh":1.34, "FR_thigh":1.34,
                           "RL_calf":-2.83, "RR_calf":-2.83, "FL_calf":-2.83, "FR_calf":-2.83}
        
    def loop(self):
        rospy.init_node('controller_quadruped', anonymous=True)
        rate = rospy.Rate(50) 

        # List of joints to control
        joints = ["RL_hip", "RR_hip", "FL_hip", "FR_hip", 
                  "RL_thigh", "RR_thigh", "FL_thigh", "FR_thigh", 
                  "RL_calf", "RR_calf", "FL_calf", "FR_calf"]

        # Create subscribers and publishers for each joint
        for joint in joints:
            command_topic = f"/joint_controller_{joint}/cmd"
        
            # Publisher for joint commands
            pub = rospy.Publisher(command_topic, MotorCmd, queue_size=1)
            self.joint_command_publishers[joint] = pub

        rospy.sleep(0.1)
        # Set up a ROS rate to manage publishing speed

        while not rospy.is_shutdown():
            for joint_name in ["FL_hip", "RL_hip", "RR_hip", "FR_hip", 
                               "RL_thigh", "RR_thigh", "FL_thigh", "FR_thigh",
                               "RL_calf", "RR_calf", "FL_calf", "FR_calf"]:#self.joints_ref.items():
                
                # Control logic: apply a simple proportional controller                    
                # Create and publish the command message
                desired_position = self.joints_ref[joint_name]
                command_msg = MotorCmd()
                command_msg.header.stamp = rospy.Time.now()
                command_msg.mode = 0x0A  # Position control mode
                command_msg.q = desired_position
                command_msg.dq = 0 
                command_msg.tau = 0 #control_effort
                command_msg.Kp = 60 # Position gain
                command_msg.Kd = 3   # Damping gain

                if joint_name in self.joint_command_publishers:
                    self.joint_command_publishers[joint_name].publish(command_msg)
                    rospy.loginfo(f"Control command for {joint_name}: Position = {self.joints_ref[joint_name]}")


            rate.sleep()  # Sleep to maintain the loop rate at 100 Hz

if __name__ == '__main__':
    controller = Controller()
    controller.loop()