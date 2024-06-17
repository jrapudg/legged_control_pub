#!/usr/bin/env python
import rospy
import numpy as np
from unitree_legged_msgs.msg import GoalState#, QuadrupedState
from nav_msgs.msg import Odometry

class GoalPub:
    def __init__(self):
        # self.goal_ori = [[1, 0, 0, 0], 
        #                  [1, 0, 0, 0]]

       
        # self.goal_pos = [[0.6, 0, 0.2],
        #                  [0.6, 0, 0.2]]
        
        # self.goal_pos = [[-3.3, -1.15, 0.2],
        #                  [-3.3, -1.15, 0.2]]
        
        # self.goal_ori = [[1, 0, 0, 0], 
        #                  [1, 0, 0, 0]]

        
        # self.goal_pos = [[0, 0, 0.2],
        #                  [0, 0, 0.2]]
        
        # self.goal_ori = [[1, 0, 0, 0], 
        #                  [1, 0, 0, 0]]
        
               
        # self.goal_pos = [[1, 0, 0.2],
        #                  [1, 0, 0.2]]
        

        
        # self.goal_ori = [[1, 0, 0, 0], 
        #                  [1, 0, 0, 0]]
       
        
        # self.goal_pos = [[1, 0, 0.2],
        #                  [2, 0, 0.2], 
        #                  [2, 1, 0.2], 
        #                  [2, 2, 0.2],
        #                  [1, 2, 0.2],
        #                  [0, 2, 0.2],
        #                  [0, 1, 0.2],
        #                  [0, 0, 0.2]]
        
        # self.goal_ori = [[1, 0, 0, 0], 
        #                  [1, 0, 0, 0], 
        #                  #[0.92388,0,0,0.38268],  #45 degrees
        #                  [0.7071, 0, 0, 0.7071], 
        #                  [0.7071, 0, 0, 0.7071],
        #                  #[0.7071, 0, 0.7071, 0], 
        #                  #[0.7071, 0, 0.7071, 0],
        #                  [0, 0, 0, 1],
        #                  [0, 0, 0, 1],
        #                  [-0.7071, 0, 0, 0.7071],
        #                  [-0.7071, 0, 0, 0.7071]]
        
        self.goal_pos = [#[0.5, 0, 0.2],
                         
                         [1, 0, 0.2],
                         [1, 0, 0.2],
                         [1, 0, 0.2],

                         [1.5, 0.5, 0.2],

                         [2, 1, 0.2],
                         [2, 1, 0.2],
                         [2, 1, 0.2],
                         
                         #[2, 1.5, 0.2],

                         [2, 2, 0.2],
                         [2, 2, 0.2],
                         [2, 2, 0.2],

                         [1.5, 2.5, 0.2],

                         [1, 3, 0.2],
                         [1, 3, 0.2],
                         [1, 3, 0.2],

                         #0.5, 3, 0.2],

                         [0, 3, 0.2],
                         [0, 3, 0.2], 
                         [0, 3, 0.2], 

                         [-0.5, 2.5, 0.2],

                         [-1, 2, 0.2], 
                         [-1, 2, 0.2],
                         [-1, 2, 0.2],

                         #[-1, 1.5, 0.2],

                         [-1, 1, 0.2],
                         [-1, 1, 0.2],
                         [-1, 1, 0.2],

                         [-0.5, 0.5, 0.2],

                         [0, 0, 0.2],
                         [0, 0, 0.2],
                         [0, 0, 0.2]]
        
        self.goal_ori = [#[1, 0, 0, 0], #0 degrees
                         
                         [1, 0, 0, 0], #0 degrees
                         [0.9808,0,0,0.1951], #22.5 degrees
                         [0.92388, 0, 0, 0.38268], #45 degrees

                         [0.92388, 0, 0, 0.38268], #45 degrees

                         [0.92388, 0, 0, 0.38268], #45 degrees
                         [0.8315,0,0,0.5556], #67.5 degrees
                         [0.7071, 0, 0, 0.7071], #90 degrees

                         #[0.7071, 0, 0, 0.7071], #90 degrees
                         
                         [0.7071, 0, 0, 0.7071], #90 degrees
                         [0.5556,0,0,0.8315], #112.5 degrees
                         [0.38268, 0, 0, 0.92388], #135 degrees

                         [0.38268, 0, 0, 0.92388], #135 degrees

                         [0.38268, 0, 0, 0.92388], #135 degrees
                         [0.1951,0,0,0.9808], #157.5 degrees
                         [0, 0, 0, 1], #180 degrees

                         #[0, 0, 0, 1], #180 degrees

                         [0, 0, 0, 1], #180 degrees
                         [-0.1951,0,0,0.9808], #202.5 degrees
                         [-0.38268, 0, 0, 0.92388], #225 degrees

                         [-0.38268, 0, 0, 0.92388], #225 degrees

                         [-0.38268, 0, 0, 0.92388], #225 degrees
                         [-0.5556,0,0,0.8315], #247.5 degrees
                         [-0.7071, 0, 0, 0.7071], #270 degrees

                         #[-0.7071, 0, 0, 0.7071], #270 degrees

                         [-0.7071, 0, 0, 0.7071], #270 degrees
                         [-0.8315,0,0,0.5556], #292.5 degrees
                         [-0.92388, 0, 0, 0.38268], #315 degrees

                         [-0.92388, 0, 0, 0.38268], #315 degrees

                         [-0.92388, 0, 0, 0.38268], #315 degrees
                         [-0.9808,0,0,0.1951],
                         [1, 0, 0, 0]] #337.5 degrees        
            
        self.index = 0
        self.body_pos = [0,0,0]
        self.body_ori = [1,0,0,0]
        self.body_vel = [0,0,0]
        self.body_ang_vel = [0,0,0]
        
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

    def loop(self):
        rospy.init_node('goal_node')
        rate = rospy.Rate(1)
        self.pub = rospy.Publisher('/quadruped/goal', GoalState, queue_size=10)
        #body_topic = "/quadruped/body_estimate"
        #rospy.Subscriber(body_topic, QuadrupedState, self.body_state_callback)
        
        #body_topic = "/ground_truth/state"
        #body_topic = "/mocap_node/Go1_body/Odom"
        body_topic = "/odom"
        rospy.Subscriber(body_topic, Odometry, self.odom_callback)
        
        while not rospy.is_shutdown():
            # Create a Float32MultiArray message

            dist_to_goal = np.linalg.norm(np.array(self.goal_pos[self.index]) - np.array(self.body_pos))
            quat_dist = 1 - np.abs(np.dot(self.goal_ori[self.index], - np.array(self.body_ori)))
            rospy.loginfo(f"Distance to goal {dist_to_goal} ")
            rospy.loginfo(f"Quat to goal {quat_dist} ")
            if dist_to_goal < 0.18 and quat_dist < 0.002:
                self.index = (self.index + 1)%len(self.goal_pos)
            goal_msg = GoalState()
            goal_msg.body_pos = self.goal_pos[self.index]
            goal_msg.body_ori = self.goal_ori[self.index]

            self.pub.publish(goal_msg)
            rospy.loginfo(f"Goal body pose {self.goal_pos[self.index]}, Goal body orientation {self.goal_ori[self.index]} ")
            #rospy.loginfo(f"Estimated body pose {self.body_pos}, Estimated body orientation {self.body_ori} ")
            
            rate.sleep()
            #self.index = (self.index + 1)%self.goals
            

if __name__ == '__main__':
    scheduler = GoalPub()
    scheduler.loop()