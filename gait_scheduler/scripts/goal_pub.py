#!/usr/bin/env python
import rospy
import numpy as np
from unitree_legged_msgs.msg import GoalState#, QuadrupedState
from nav_msgs.msg import Odometry

class GoalPub:
    def __init__(self):
        # # Stand-up task
        # self.goal_pos = [[0.0, 0.0, 0.2],
        #                  [0.0, 0.0, 0.2]]
        
        # self.goal_ori = [[1, 0, 0, 0],
        #                  [1, 0, 0, 0]]

        # # Gazebo Straight line task
        # self.goal_pos = [[0, 0.0, 0.27],
        #                  #[-0.8, 0.5, 0.27],
        #                  [1.0, 0.0, 0.27],
        #                  [2.0, 0.0, 0.27],
        #                  [1.0, 0.0, 0.27],
        #                  [0.0, 0.0, 0.27]]
        
        # self.goal_ori = [[1, 0, 0, 0],
        #                  #[1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0]]
        
        # HW Straight line task
        self.goal_pos = [[-2.8, 0.5, 0.27],
                         [-1.6, 0.5, 0.27],
                         #[-0.8, 0.5, 0.27],
                         [0.4, 0.5, 0.27],
                         [-0.8, 0.5, 0.27],
                         [-1.8, 0.5, 0.27]]
        
        self.goal_ori = [[1, 0, 0, 0],
                         #[1, 0, 0, 0],
                         [1, 0, 0, 0],
                         [1, 0, 0, 0],
                         [1, 0, 0, 0],
                         [1, 0, 0, 0]]

        # # Straight line task
        # self.goal_pos = [[0.0, 0.0, 0.27],
        #                  [0.3, 0.0, 0.27],
        #                  [0.0, 0.0, 0.27],
        #                  [-0.3, 0.0, 0.27]]
        
        # self.goal_ori = [[1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0]]

        # self.goal_pos = [[0.0, 0.0, 0.2],
        #                  [0.0, 0.0, 0.2],
        #                  [1.0, 0.0, 0.2],
        #                  [2.0, 0.0, 0.2],
        #                  [3.0, 0.0, 0.2],
        #                  [4.0, 0.0, 0.2],
        #                  [5.0, 0.0, 0.2]]
        
        # self.goal_ori = [[1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [1, 0, 0, 0]]
        
        # # Octagon Task
        # self.goal_pos = [#[0.5, 0, 0.2],
                         
        #                  [1, 0, 0.27],
        #                  [1, 0, 0.27],
        #                  [1, 0, 0.27],

        #                  [1.5, 0.5, 0.27],

        #                  [2, 1, 0.27],
        #                  [2, 1, 0.27],
        #                  [2, 1, 0.27],
                         
        #                  #[2, 1.5, 0.2],

        #                  [2, 2, 0.27],
        #                  [2, 2, 0.27],
        #                  [2, 2, 0.27],

        #                  [1.5, 2.5, 0.27],

        #                  [1, 3, 0.27],
        #                  [1, 3, 0.27],
        #                  [1, 3, 0.27],

        #                  #0.5, 3, 0.2],

        #                  [0, 3, 0.27],
        #                  [0, 3, 0.27], 
        #                  [0, 3, 0.27], 

        #                  [-0.5, 2.5, 0.27],

        #                  [-1, 2, 0.27], 
        #                  [-1, 2, 0.27],
        #                  [-1, 2, 0.27],

        #                  #[-1, 1.5, 0.2],

        #                  [-1, 1, 0.27],
        #                  [-1, 1, 0.27],
        #                  [-1, 1, 0.27],

        #                  [-0.5, 0.5, 0.27],

        #                  [0, 0, 0.27],
        #                  [0, 0, 0.27],
        #                  [0, 0, 0.27]]
        
        # self.goal_ori = [#[1, 0, 0, 0], #0 degrees
                         
        #                  [1, 0, 0, 0], #0 degrees
        #                  [0.9808,0,0,0.1951], #22.5 degrees
        #                  [0.92388, 0, 0, 0.38268], #45 degrees

        #                  [0.92388, 0, 0, 0.38268], #45 degrees

        #                  [0.92388, 0, 0, 0.38268], #45 degrees
        #                  [0.8315,0,0,0.5556], #67.5 degrees
        #                  [0.7071, 0, 0, 0.7071], #90 degrees

        #                  #[0.7071, 0, 0, 0.7071], #90 degrees
                         
        #                  [0.7071, 0, 0, 0.7071], #90 degrees
        #                  [0.5556,0,0,0.8315], #112.5 degrees
        #                  [0.38268, 0, 0, 0.92388], #135 degrees

        #                  [0.38268, 0, 0, 0.92388], #135 degrees

        #                  [0.38268, 0, 0, 0.92388], #135 degrees
        #                  [0.1951,0,0,0.9808], #157.5 degrees
        #                  [0, 0, 0, 1], #180 degrees

        #                  #[0, 0, 0, 1], #180 degrees

        #                  [0, 0, 0, 1], #180 degrees
        #                  [-0.1951,0,0,0.9808], #202.5 degrees
        #                  [-0.38268, 0, 0, 0.92388], #225 degrees

        #                  [-0.38268, 0, 0, 0.92388], #225 degrees

        #                  [-0.38268, 0, 0, 0.92388], #225 degrees
        #                  [-0.5556,0,0,0.8315], #247.5 degrees
        #                  [-0.7071, 0, 0, 0.7071], #270 degrees

        #                  #[-0.7071, 0, 0, 0.7071], #270 degrees

        #                  [-0.7071, 0, 0, 0.7071], #270 degrees
        #                  [-0.8315,0,0,0.5556], #292.5 degrees
        #                  [-0.92388, 0, 0, 0.38268], #315 degrees

        #                  [-0.92388, 0, 0, 0.38268], #315 degrees

        #                  [-0.92388, 0, 0, 0.38268], #315 degrees
        #                  [-0.9808,0,0,0.1951],
        #                  [1, 0, 0, 0]] #337.5 degrees        
        self.index = 0
        self.body_pos = [0,0,0]
        self.body_ori = [1,0,0,0]
        self.body_xy = [0, 0]
        self.body_z = [0]
        
    def body_state_callback(self, data):
        # Store the latest state data
        self.body_pos = data.body_pos
        self.body_ori = data.body_ori

    # def odom_callback(self, data):
    #     # Store the latest state data
    #     #print("odom_callback")
    #     self.body_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    #     self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
        #self.body_vel = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        #self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]   
    def ori_callback(self, data):
        # Store the latest state data
        #print("odom_callback")
        self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
    
    def pos_callback(self, data):
        # Store the latest state data
        #print("odom_callback")
        self.body_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

    def pos_xy_callback(self, data):
        self.body_xy = [data.pose.pose.position.x, data.pose.pose.position.y]
    
    def pos_z_callback(self, data):
        self.body_z = [data.pose.pose.position.z]

    def loop(self):
        rospy.init_node('goal_node')
        rate = rospy.Rate(1)
        self.pub = rospy.Publisher('/quadruped/goal', GoalState, queue_size=10)
        #body_topic = "/quadruped/body_estimate"
        #rospy.Subscriber(body_topic, QuadrupedState, self.body_state_callback)
        
        mocap_topic = "/mocap_node/Go1_body/Odom"
        #pos_topic = "/ground_truth/state"
        odom_topic = "/odom"
        gazebo_topic = "/ground_truth/state"

        # # Gazebo Simulation
        # rospy.Subscriber(gazebo_topic, Odometry, self.ori_callback)
        # rospy.Subscriber(gazebo_topic, Odometry, self.pos_xy_callback)
        # rospy.Subscriber(odom_topic, Odometry, self.pos_z_callback)

        # Unitree HW
        rospy.Subscriber(mocap_topic, Odometry, self.ori_callback)
        rospy.Subscriber(mocap_topic, Odometry, self.pos_xy_callback)
        rospy.Subscriber(odom_topic, Odometry, self.pos_z_callback)
   
        
        while not rospy.is_shutdown():
            self.body_pos = [self.body_xy[0], self.body_xy[1], self.body_z[0]]
            # Create a Float32MultiArray message
            rospy.loginfo(f"Calculating ...")
            dist_to_goal = np.linalg.norm(np.array(self.goal_pos[self.index]) - np.array(self.body_pos))
            quat_dist = 1 - np.abs(np.dot(self.goal_ori[self.index], - np.array(self.body_ori)))
            rospy.loginfo(f"Distance to goal {dist_to_goal} ")
            rospy.loginfo(f"Quat to goal {quat_dist} ")
            if dist_to_goal < 0.1 and quat_dist < 0.005:
                self.index = (self.index + 1)%len(self.goal_pos)
            goal_msg = GoalState()
            goal_msg.body_pos = self.goal_pos[self.index]
            goal_msg.body_ori = self.goal_ori[self.index]

            self.pub.publish(goal_msg)
            rospy.loginfo(f"Goal body pose {self.goal_pos[self.index]}, Goal body orientation {self.goal_ori[self.index]} ")
            rospy.loginfo(f"Ground Truth body pose {self.body_pos}, Ground Truth body orientation {self.body_ori} ")
            
            rate.sleep()
            #self.index = (self.index + 1)%self.goals
            

if __name__ == '__main__':
    scheduler = GoalPub()
    scheduler.loop()