#!/usr/bin/env python
#coding:utf-8
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import time

class OdometryNode:
    
    #设置发布者，发布名为/odom的topic，类型为nav_msgs/Odometry
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
    #类实例化的初始化方法
    def __init__(self):
        # 初始化参数
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        #设置更新频率，每0.05s调用回调函数
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz
        #定义广播器
        self.tf_pub = tf2_ros.TransformBroadcaster()
        #设置订阅者，订阅到gazebo/link_states则调用回调函数
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)
    def sub_robot_pose_update(self, msg):
        #值'wheeltec::base_footprint'的位置
        try:
            arrayIndex = msg.name.index('wheeltec::base_footprint')          
        except ValueError as e:
            #搜索列表中不存在的值引发的异常，即等待Gazebo启动成功
            pass
        else:
            #值存在，提取当前信息
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        #获取当前时间
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        #赋值
        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_footprint'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        #参照turtlebot_node中odom协方差设置
        cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
			      0, 1e-3, 0, 0, 0, 0,
			      0, 0, 1e6, 0, 0, 0,
			      0, 0, 0, 1e6, 0, 0,
			      0, 0, 0, 0, 1e6, 0,
			      0, 0, 0, 0, 0, 1e3]
        cmd.twist.covariance =[1e-3, 0, 0, 0, 0, 0, 
                               0, 1e-3, 1e-9, 0, 0, 0,
                               0, 0, 1e6, 0, 0, 0,
                               0, 0, 0, 1e6, 0, 0,
                               0, 0, 0, 0, 1e6, 0,
                               0, 0, 0, 0, 0, 1e3]

        #发布odom话题
        self.pub_odom.publish(cmd)
        if cmd.pose.pose.orientation.w == 0.0:
            return
        #tf变换
        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        #发布tf变换
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    #初始化节点
    rospy.init_node("gazebo_odometry_node")
    #类的实例化
    node = OdometryNode()
    rospy.spin()


