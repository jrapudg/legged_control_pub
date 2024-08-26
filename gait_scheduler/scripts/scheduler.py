#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import MultiArrayDimension, Float32MultiArray, Header
from unitree_legged_msgs.msg import GaitState

class GaitScheduler:
    def __init__(self, gait_path = '/home/ControlUser/legged_ctrl_ws/src/scripts/gaits/walking_gait_NORMAL_HIGHER.tsv', phase_time = 0):
        # Load the configuration file
        with open(gait_path, 'r') as file:
            gait_array = np.loadtxt(file, delimiter='\t')
        
        # Load model
        self.gait = gait_array
        self.phase_length = gait_array.shape[1]
        self.phase_time = phase_time
        self.indices = np.arange(self.phase_length)

        self.pub = None
        
    def roll(self):
        self.phase_time += 1
        self.indices = np.roll(self.indices, -1)
    
    def get_current_ref(self):
        return self.gait[:, self.phase_time]

    def loop(self):
        rospy.init_node('gait_scheduler_node')
        rate = rospy.Rate(80)
        self.pub = rospy.Publisher('/quadruped/gait', GaitState, queue_size=10)

        while not rospy.is_shutdown():
            # Create a Float32MultiArray message
            gait_state_msg = GaitState()
            gait_array = Float32MultiArray()
            indices_array = Float32MultiArray()
            
            gait = self.gait[:, self.indices]
            # Flatten the numpy array and fill in the message data
            gait_array.data = gait.ravel()
            gait_array.layout.dim.append(MultiArrayDimension())
            gait_array.layout.dim[0].label = "height"
            gait_array.layout.dim[0].size = gait.shape[0]
            gait_array.layout.dim[0].stride = gait.shape[0] * gait.shape[1]
            gait_array.layout.dim.append(MultiArrayDimension())
            gait_array.layout.dim[1].label = "width"
            gait_array.layout.dim[1].size = gait.shape[1]
            gait_array.layout.dim[1].stride = gait.shape[1]

            indices_array.data = self.indices.ravel()
            #indices_array.layout.dim.append(MultiArrayDimension())
            #indices_array.layout.dim[0].label = "width"
            #indices_array.layout.dim[0].size = self.indices.shape[0]
            #indices_array.layout.dim[0].stride = self.indices.shape[0]

            gait_state_msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
            gait_state_msg.gait = gait_array
            gait_state_msg.indices = indices_array

            gait_state_msg.current_joints_pos = self.gait[:, self.indices[0]]

            self.pub.publish(gait_state_msg)
            #rospy.loginfo(f"Phase tick {self.indices[0]}")
            self.roll()
            rate.sleep()

if __name__ == '__main__':
    scheduler = GaitScheduler()
    scheduler.loop()