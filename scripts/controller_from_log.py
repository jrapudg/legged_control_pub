#!/usr/bin/env python

import rospy
import numpy as np
from unitree_legged_msgs.msg import MotorCmd
from nav_msgs.msg import Odometry

class Controller:
    def __init__(self):
        self.joint_command_publishers = {}
        # self.joints_ref = {"RL_hip":0.3, "RR_hip":-0.3, "FL_hip":0.3, "FR_hip":-0.3, 
        #                   "RL_thigh":0.9, "RR_thigh":0.9, "FL_thigh":0.9, "FR_thigh":0.9, 
        #                   "RL_calf":-1.8, "RR_calf":-1.8, "FL_calf":-1.8, "FR_calf":-1.8}
        self.joints_ref_ = np.loadtxt('mujoco_logs/walking_straight.tsv', delimiter='\t')
    
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

        rospy.sleep(1)
        
        # Iteration counter
        iteration_counter = 0

        while not rospy.is_shutdown():
            # Log the iteration number
            rospy.loginfo(f"Iteration number: {iteration_counter}")
            
            for i, joint_name in enumerate(["FR_hip", "FR_thigh", "FR_calf",
                                            "FL_hip", "FL_thigh", "FL_calf",
                                            "RR_hip", "RR_thigh", "RR_calf",
                                            "RL_hip", "RL_thigh", "RL_calf"]):                                               
                                               
                
                # Control logic: apply a simple proportional controller                    
                # Create and publish the command message
                desired_position = self.joints_ref_[iteration_counter][i]
                command_msg = MotorCmd()
                command_msg.mode = 0x0A  # Position control mode
                command_msg.q = desired_position
                command_msg.dq = 0 
                command_msg.tau = 0 #control_effort
                command_msg.Kp = 60 # Position gain
                command_msg.Kd = 3   # Damping gain

                if joint_name in self.joint_command_publishers:
                    self.joint_command_publishers[joint_name].publish(command_msg)
                    rospy.loginfo(f"Control command for {joint_name}: Position = {self.joints_ref_[iteration_counter][i]}")

            # Increment the counter and reset if it reaches 50
            iteration_counter = (iteration_counter + 1) % 501
            rate.sleep()  # Sleep to maintain the loop rate at 100 Hz

if __name__ == '__main__':
    controller = Controller()
    controller.loop()