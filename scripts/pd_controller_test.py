#!/usr/bin/env python
import sys

sys.path.append('../../')
import rospy
import numpy as np
from control.pd import JointServoPD
from unitree_legged_msgs.msg import MotorState, MotorCmd

#joints_ref = {"RL_hip":0, "RR_hip":0, "FL_hip":0, "FR_hip":0, 
#              "RL_thigh":0, "RR_thigh":0, "FL_thigh":0, "FR_thigh":0, 
#              "RL_calf":0, "RR_calf":0, "FL_calf":0, "FR_calf":0}

class Controller:
    def __init__(self):
        # self.joints_ref = {"RL_hip":0, "RR_hip":0, "FL_hip":0, "FR_hip":0, 
        #                   "RL_thigh":0.9, "RR_thigh":0.9, "FL_thigh":0.9, "FR_thigh":0.9, 
        #                   "RL_calf":-1.8, "RR_calf":-1.8, "FL_calf":-1.8, "FR_calf":-1.8}
        # self.joints_ref = {"RL_hip":0.073, "RR_hip":0.073, "FL_hip":0.073, "FR_hip":0.073, 
        #                    "RL_thigh":1.34, "RR_thigh":1.34, "FL_thigh":1.34, "FR_thigh":1.34, 
        #                    "RL_calf":-2.83, "RR_calf":-2.83, "FL_calf":-2.83, "FR_calf":-2.83}
        self.joints_ref = {"RL_hip":0.4, "RR_hip":-0.4, "FL_hip":0.4, "FR_hip":-0.4, 
                          "RL_thigh":0.9, "RR_thigh":0.9, "FL_thigh":0.9, "FR_thigh":0.9, 
                          "RL_calf":-1.7504, "RR_calf":-1.7504, "FL_calf":-1.7504, "FR_calf":-1.7504}
        self.joint_command_publishers = {}
        self.joint_states = {"RL_hip":None, "RR_hip":None, "FL_hip":None, "FR_hip":None, 
                             "RL_thigh":None, "RR_thigh":None, "FL_thigh":None, "FR_thigh":None, 
                             "RL_calf":None, "RR_calf":None, "FL_calf":None, "FR_calf":None}

    def joint_state_callback(self, data, joint_name):
        # Store the latest state data
        self.joint_states[joint_name] = data

    def loop(self):
        rospy.init_node('controller_quadruped', anonymous=True)
        rate = rospy.Rate(100) 

        # List of joints to control
        joints = ["RL_hip", "RR_hip", "FL_hip", "FR_hip", 
                  "RL_thigh", "RR_thigh", "FL_thigh", "FR_thigh", 
                  "RL_calf", "RR_calf", "FL_calf", "FR_calf"]
        
        # Create subscribers and publishers for each joint
        for joint in joints:
            state_topic = f"/go1_gazebo/{joint}_controller/state"
            command_topic = f"/go1_gazebo/{joint}_controller/command"
            
            # Subscribe to joint state
            rospy.Subscriber(state_topic, MotorState, self.joint_state_callback, joint)

            # Publisher for joint commands
            pub = rospy.Publisher(command_topic, MotorCmd, queue_size=1)
            self.joint_command_publishers[joint] = pub

        rospy.sleep(2)
        # Set up a ROS rate to manage publishing speed
        while not rospy.is_shutdown():
            print()
            for joint_name, data in self.joint_states.items():
                if data is not None:
                    # Control logic: apply a simple proportional controller
                    desired_position = self.joints_ref[joint_name]
                    control_effort = desired_position
                    # Create and publish the command message
                    command_msg = MotorCmd()
                    command_msg.mode = 0x0A  # Position control mode
                    command_msg.q = control_effort  # Assume 'q' is the command position in MotorCmd
                    command_msg.dq = 0 
                    command_msg.tau = 0 #control_effort
                    command_msg.Kp = 40  # Position gain
                    command_msg.Kd = 3    # Damping gain

                    if joint_name in self.joint_command_publishers:
                        self.joint_command_publishers[joint_name].publish(command_msg)
                        rospy.loginfo(f"Control command for {joint_name}: Position = {data.q}")

            rate.sleep()  # Sleep to maintain the loop rate at 100 Hz

if __name__ == '__main__':
    controller = Controller()
    controller.loop()