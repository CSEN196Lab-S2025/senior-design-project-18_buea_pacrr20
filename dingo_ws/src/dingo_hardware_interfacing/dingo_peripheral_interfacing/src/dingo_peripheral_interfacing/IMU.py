#!/usr/bin/env python3
import rospy
import numpy as np
import time
import time
import board
import adafruit_bno055
import math as m
from sensor_msgs.msg import Imu

class IMU:
    def __init__(self):
        #self.i2c = board.I2C()  # uses board.SCL and board.SDA
        #self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        #self.last_euler = np.array([ 0, 0, 0])
        #self.start_time = time.time()
        self.odom_message = rospy.Subscriber("/wit/imu", Imu, self.odom_callback) #added -Ethan
        

    def read_orientation(self):
        """Reads quaternion measurements from the IMU until . Returns the last read Euler angle.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        np array (3,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """
        try: 
            [yaw,pitch,roll] = self.sensor.euler
            yaw = m.radians(360-yaw) 
            pitch = m.radians(-pitch)
            roll = m.radians(roll - 0*30) # fixed offset to account for the IMU being off by 30 degrees <= I'm not sure why they added this? -Ethan
            self.last_euler = [yaw,pitch,roll]
        except:
            self.last_euler = np.array([ 0, 0, 0])
        return self.last_euler
    # Added everything below from automove
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = m.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = m.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = m.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def odom_callback(self, msg):
        # Extract orientation
        print(msg)
        self.x_quaternion = msg.orientation.x
        self.y_quaternion = msg.orientation.y
        self.z_quaternion = msg.orientation.z
        self.w_quaternion = msg.orientation.w
        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x_quaternion, self.y_quaternion, self.z_quaternion, self.w_quaternion)

        self.test_var = self.test_var + 1
        
        # Extract angular velocity
        self.angular_velocity_x = msg.angular_velocity.x
        self.angular_velocity_y = msg.angular_velocity.y
        self.angular_velocity_z = msg.angular_velocity.z

        # Extract linear acceleration
        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y
        self.linear_acceleration_z = msg.linear_acceleration.z