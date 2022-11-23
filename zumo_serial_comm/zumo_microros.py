#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
from math import pi
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32, UInt8MultiArray
from sensor_msgs.msg import LaserScan

import zumo_serial_comm.transformations as transformations# for euler to quat
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


import sys
import numpy as np

class ZumoMicroRosNode(Node):
    def __init__(self):
        super().__init__("zumo_microros")
        
        self.odom_sub = self.create_subscription(Point32, "odom_data", self.odom_data_callback, 1)
        self.prox_sub = self.create_subscription(UInt8MultiArray, "zumo_prox", self.zumo_prox_callback, 1)
        
        self.odom_pub = self.create_publisher(Odometry, "odom", 5)
        self.yaw_pub = self.create_publisher(Float32, "yaw_deg", 5)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 5)
        
        self.odom_broadcaster = TransformBroadcaster(self)
        self.tfs = TransformStamped()
        self.tfs.header.frame_id = "odom"
        self.tfs.child_frame_id = "base_link"
        self.tfs.transform.translation.z = 0.0
        
        now_stamp = self.get_clock().now().to_msg()
        self.prev_time = now_stamp
        self.error_time = now_stamp

        self.enc_total = 0.0
        
        self.dist_sum = 0.0
        self.time_sum = 0.0
        self.vx = 0.0

        self.bot_deg_prev = 0.0
        self.bot_deg = 0.0
        self.botx = 0.0
        self.boty = 0.0
        
        self.gyro_sum = 0.0
        self.gyro_count = 0
        self.gyro_bias_rad = -0.40*pi/180
        self.gyro_scale = 0.00875 # TODO: set this in the esp32 code
        
        #self.timer = self.create_timer(1./20., self.update_odom)
        
        self.get_logger().info("Started Zumo Micro ROS Karl")
    
    def dt_to_sec(self, stampA, stampB):
        return stampA.sec + stampA.nanosec * 10**-9 - stampB.sec - stampB.nanosec * 10**-9
        
    def other_callback(self, data):
        print("ODOM2")
        
    def zumo_prox_callback(self, data):
        #print("prox data: %d, %d, %d, %d" % (data.data[0], data.data[1], data.data[2], data.data[3]))
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -pi/2
        #scan.angle_max = pi/2
        scan.angle_increment = pi/3.0
        scan.time_increment = 0.01
        scan.scan_time = 0.1
        scan.range_max = 0.4
        scan.ranges = [0.,0.,0.,0.]
        scan.ranges[0] = (7 - data.data[3]) / 15.0
        scan.ranges[1] = (7 - data.data[2] ) / 15.0
        scan.ranges[2] = (7 - data.data[1] ) / 15.0
        scan.ranges[3] = (7 - data.data[0] ) / 15.0
        self.scan_pub.publish(scan)
    
    def odom_data_callback(self, data):
        t2 = self.get_clock().now().to_msg()
        t1 = self.prev_time
        dt = self.dt_to_sec(t2,t1)
        
        BOT_WIDTH = 0.09 #meters
        COUNTS_PER_METER = 5400.0
        
        # Process gyro z
        gyro_thresh_dps = 0.2
        g_bias_dps = self.gyro_bias_rad*180/pi
        MAX_DTHETA_GYRO_deg = 100.0
            
        # Read encoders and raw_gyro_z data 
        delta_enc_left = data.x
        delta_enc_right = data.y
        # raw_gyro_z
        #gyroz_raw_dps = raw_gyro_z * self.gyro_scale
        gyroz_raw_dps = data.z
        #print("odom data %.1f, %.1f, %.1f" % (delta_enc_left, delta_enc_right, gyroz_raw_dps))
        
        if(abs(gyroz_raw_dps-g_bias_dps) < gyro_thresh_dps):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = gyroz_raw_dps-g_bias_dps
            dtheta_gyro_deg = gz_dps*dt
        
        # Update odom
        
        delta_enc_counts = float(delta_enc_left + delta_enc_right)/2.0
        self.enc_total = self.enc_total + delta_enc_counts
        
        dmeters = float(delta_enc_left + delta_enc_right)/2.0 / COUNTS_PER_METER
        
        dtheta_enc_deg = float(delta_enc_right - delta_enc_left) / COUNTS_PER_METER / BOT_WIDTH * 180.0 / pi

        if(abs(dtheta_gyro_deg) > MAX_DTHETA_GYRO_deg):
            print('no gyro')
            dtheta_deg = dtheta_enc_deg
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg

        #update bot position
        self.bot_deg = self.bot_deg + dtheta_deg
        avg_cos = (np.cos(self.bot_deg*pi/180) + np.cos(self.bot_deg_prev*pi/180) ) / 2
        avg_sin = (np.sin(self.bot_deg*pi/180) + np.sin(self.bot_deg_prev*pi/180) ) / 2
        self.bot_deg_prev = self.bot_deg
        dx = dmeters*avg_cos
        dy = dmeters*avg_sin
        #dx = dmeters*np.cos(self.bot_deg*pi/180)
        #dy = dmeters*np.sin(self.bot_deg*pi/180)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        #print 'bot x,y,deg: ', self.bot.botx, self.bot.boty, self.bot.bot_deg
        
        # update bot linear x velocity every 150 msec
        # need to use collections deque, then push and pop, moving average
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            self.dist_sum = 0.0
            self.time_sum = 0.0
        
        odom_quat = transformations.quaternion_from_euler(0, 0, self.bot_deg*pi/180.0)
        
        self.tfs.header.stamp = t2
        self.tfs.transform.translation.x = self.botx
        self.tfs.transform.translation.y = self.boty
        self.tfs.transform.rotation.x = odom_quat[0]
        self.tfs.transform.rotation.y = odom_quat[1]
        self.tfs.transform.rotation.z = odom_quat[2]
        self.tfs.transform.rotation.w = odom_quat[3]
        self.odom_broadcaster.sendTransform(self.tfs)
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        #odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))
        odom.pose.pose.position.x = self.botx
        odom.pose.pose.position.y = self.boty
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = self.tfs.transform.rotation.x
        odom.pose.pose.orientation.y = self.tfs.transform.rotation.y
        odom.pose.pose.orientation.z = self.tfs.transform.rotation.z
        odom.pose.pose.orientation.w = self.tfs.transform.rotation.w

        # set the velocity
        odom.child_frame_id = "base_link"
        if dt > 0:
            out_gz_dps = dtheta_deg / dt
        else:
            out_gz_dps = 0
        #odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, out_gz_dps*pi/180.0))
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = out_gz_dps*pi/180.0

        # publish the message
        self.odom_pub.publish(odom)
        
        yaw_msg = Float32()
        yaw_msg.data = self.bot_deg
        self.yaw_pub.publish(yaw_msg)
        
        self.prev_time = t2


def main(args=None):
    rclpy.init(args=args)
    node = ZumoMicroRosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
