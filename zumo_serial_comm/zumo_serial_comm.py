#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
from math import pi
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Imu

import zumo_serial_comm.transformations as transformations# for euler to quat
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


import serial
import sys
import numpy as np

class MicroSerial():

    def __init__(self):
        #self.lock = lock
        self.serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=2)
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str.encode())
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str

class Controller():
    
    def __init__(self, micro):
        self.micro = micro
        self.speed = 0.0
        self.curv = 0.0

    def write_speed_curv(self, speed_in, curv_in):
        #  A1/1/<speed_byte> <curv_byte>
        self.speed = speed_in # m/s
        self.curv = curv_in # rad/sec
        #print('speed, curv', speed_in, curv_in)
        raw_speed = int(speed_in*100.0)+120
        raw_omega = int(curv_in*180.0/pi)+120
        speed_byte = bytes([ raw_speed & 0xff]) #-120 to 120 cm/sec
        curv_byte = bytes([ (raw_omega) & 0xff]) # -120 to 120 deg/sec
        #print('speed_byte, curv_byte', speed_byte, curv_byte)
        
        self.micro.safe_write('A1/1/' + str(raw_speed) + '/' + str(raw_omega) + '/')
        return

class ZumoSerialNode(Node):
    def __init__(self):
        super().__init__("zumo_serial")
        
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.drive_callback, 1)
        
        self.odom_pub = self.create_publisher(Odometry, "odom", 5)
        self.yaw_pub = self.create_publisher(Float32, "yaw_deg", 5)
        self.encLeft_pub = self.create_publisher(Int16, 'encLeft', 5)
        self.encRight_pub = self.create_publisher(Int16, 'encRight',5)
        
        self.odom_broadcaster = TransformBroadcaster(self)
        self.tfs = TransformStamped()
        self.tfs.header.frame_id = "odom"
        self.tfs.child_frame_id = "base_link"
        self.tfs.transform.translation.z = 0.0
        
        self.micro = MicroSerial()
        self.controller = Controller(self.micro)
        self.speed = 0.0
        self.curv = 0.0
        
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
        self.gyro_bias_rad = -0.42*pi/180
        self.gyro_scale = 0.0091
        
        self.timer = self.create_timer(1./20., self.update_odom)
        
        self.get_logger().info("Started Zumo Serial ROS Comm")
    
    def dt_to_sec(self, stampA, stampB):
        return stampA.sec + stampA.nanosec * 10**-9 - stampB.sec - stampB.nanosec * 10**-9
        
    def drive_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        max_speed = 1.2
        max_omega = 120*pi/180.0
        
        if v > max_speed:
            v = max_speed
        elif v < -max_speed:
            v = -max_speed
        
        if w > max_omega:
            w = max_omega
        elif w < -max_omega:
            w = -max_omega
        
        self.speed = v
        self.curv = w
        
        self.controller.write_speed_curv(self.speed, self.curv)
    
    def update_odom(self):
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
        try: 
            self.micro.safe_write('A3/4/')
            s = self.micro.safe_read()
            delta_enc_left = int(s)
            s = self.micro.safe_read()
            delta_enc_right = int(s)
            s = self.micro.safe_read()
            raw_gyro_z = float(s)
        except:
            delta_enc_left = 0
            delta_enc_right = 0
            raw_gyro_z = 0
            error_delta_time = self.dt_to_sec(t2, self.error_time)
            if(error_delta_time > 1.0):
                print('read enc, gryo error')
                print("Unexpected Error:", sys.exc_info()[0])
        finally:
            a=0
            
        gyroz_raw_dps = raw_gyro_z * self.gyro_scale
        
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
        
        enc_msg = Int16()
        enc_msg.data = delta_enc_left
        self.encLeft_pub.publish(enc_msg)
        enc_msg.data = delta_enc_right
        self.encRight_pub.publish(enc_msg)
        
        self.prev_time = t2


def main(args=None):
    rclpy.init(args=args)
    node = ZumoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()