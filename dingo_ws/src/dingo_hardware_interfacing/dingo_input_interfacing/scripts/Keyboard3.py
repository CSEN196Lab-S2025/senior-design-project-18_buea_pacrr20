import rospy
import sys, signal
import os
from sensor_msgs.msg import Joy
import threading
import time
import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
from dingo_servo_interfacing import HardwareInterface
from dingo_servo_interfacing.ServoCalibrationDefinition import motor_config
from sensor_msgs.msg import Imu # Added this in -Ethan


class Keyboard:
    def __init__(self):
        self.used_keys = ['w', 'a', 's', 'd', '1', '2', '7', '8', '9', '0', "backspace", "up", "down", "left", "right", "space"]
        self.speed_multiplier = 1
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)

        self.odom_message = rospy.Subscriber("/wit/imu", Imu) # Added this as well
        self.linear_accelerations = [self.odom_message.linear_acceleration.x,self.odom_message.linear_acceleration.y,self.odom_message.linear_acceleration.z] # Potential Line

        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0., 0., 0., 0., 0., 0., 0., 0.]
        self.current_joy_message.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.start_keyboard_listener()
        self.stopped = 1
        self.Dingo = motor_config()
        self.offsets = np.array([[66, 110, 118, 80],[ 7,   -8,  11, 18],[ -5,   0,  -5,  -7]])
        #self.hardware_interface = HardwareInterface(self.linkage)

    def start_keyboard_listener(self):
        keyboard_thread = threading.Thread(target=self.start_listen_keyboard)
        keyboard_thread.daemon = True
        keyboard_thread.start()

    def start_listen_keyboard(self):
        listen_keyboard(on_press=self.on_press, on_release=self.on_release)

    def on_press(self,key):

        msg = self.current_joy_message
        print(key)


        if key == 'w' or key == 'W':
            msg.axes[1] = 0.5*self.speed_multiplier
            msg.axes[3] = 0.1*self.speed_multiplier
            # msg.axes[0] = -0.1*self.speed_multiplier
        elif key == 's' or key == 'S':
            msg.axes[1] = -0.5*self.speed_multiplier
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.5*self.speed_multiplier
        elif key == 'd' or key == 'D':
            msg.axes[0] = -0.5*self.speed_multiplier
        elif key == '1':
            msg.buttons[5] = 1
        elif key == '2':
            msg.buttons[0] = 1
        elif key == "backspace":
            msg.buttons[4] = 1
        elif key == "up":
            msg.axes[4] = 0.5*self.speed_multiplier
        elif key == "down":
            msg.axes[4] = -0.5*self.speed_multiplier
        elif key == "left":
            msg.axes[3] = 0.5*self.speed_multiplier
        elif key == "right":
            msg.axes[3] = -0.5*self.speed_multiplier
        elif key == '0':
            msg.axes[7] = 1
        elif key == '9':
            msg.axes[7] = -1
        elif key == '8':
            msg.axes[6] = 1
        elif key == '7':
            msg.axes[6] = -1
        else: return
        self.current_joy_message = msg
        return

    def on_release(self, key):


        msg = self.current_joy_message


        if key == 'w' or key == 'W':
            msg.axes[1] = 0.0
            msg.axes[3] = 0.0
            msg.axes[0] = 0.0
        elif key == 's' or key == 'S':
            msg.axes[1] = 0.0
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.0
        elif key == 'd' or key == 'D':
            msg.axes[0] = 0.0
        elif key == '1':
            msg.buttons[5] = 0
        elif key == '2':
            msg.buttons[0] = 0
        elif key == "backspace":
            msg.buttons[4] = 0
        elif key == "up":
            msg.axes[4] = 0.0
        elif key == "down":
            msg.axes[4] = 0.0
        elif key == "left":
            msg.axes[3] = 0.0
        elif key == "right":
            msg.axes[3] = 0.0
        elif key == '0':
            msg.axes[7] = 0
        elif key == '9':
            msg.axes[7] = 0
        elif key == '8':
            msg.axes[6] = 0
        elif key == '7':
            msg.axes[6] = 0
        elif key == "space":
            self.pause()
            self.lidar_scan()
        
        self.current_joy_message = msg

    def publish_current_command(self):
        
        self.current_joy_message.header.stamp = rospy.Time.now()
        self.joystick_message_pub.publish(self.current_joy_message)

    def pause(self):
        print("pausing")
        #self.hardware_interface.set_actuator_postions([])
        self.move_legs([[0,40,115],[0,40,115],[0,40,115],[0,40,115]])
    
    def lidar_scan(self):
        print("lidar scanning")
        time.sleep(5) # pause 5 seconds

    def move_legs(self,pos):
    #if servo_name == "fr" or servo_name == "all":
        #try:
        self.Dingo.moveAbsAngle(self.Dingo.front_right_hip,self.offsets[0,0]+pos[0][0])
        print(self.offsets[0,0]+pos[0][0])
        #except:
        #    print("front_right_hip error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.front_right_upper,self.offsets[1,0]+pos[0][1])
            print(self.offsets[1,0]+pos[0][1])
        except:
            print("front_right_upper error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.front_right_lower,self.offsets[2,0]+pos[0][2])
            print(self.offsets[2,0]+pos[0][2])
        except:
            print("front_right_lower error caught")
        #timeSleep(0.04)
    #if servo_name == "fl" or servo_name == "all":
        try:
            self.Dingo.moveAbsAngle(self.Dingo.front_left_hip  ,self.offsets[0,1]+pos[1][0])
            print(self.offsets[0,1]+pos[1][0])
        except:
            print("front_left_hip error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.front_left_upper,self.offsets[1,1]+pos[1][1])
            print(self.offsets[1,1]+pos[1][1])
        except:
            print("front_left_upper error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.front_left_lower,self.offsets[2,1]+pos[1][2])
            print(self.offsets[2,1]+pos[1][2])
        except:
            print("front_left_lower error caught")
        #timeSleep(0.04)
    #if servo_name == "br" or servo_name == "all":
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_right_hip   ,self.offsets[0,2]+pos[2][0])
            print(self.offsets[0,2]+pos[2][0])
        except:
            print("back_right_hip error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_right_upper ,self.offsets[1,2]+pos[2][1])
            print(self.offsets[1,2]+pos[2][1])
        except:
            print("back_right_upper error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_right_lower ,self.offsets[2,2]+pos[2][2])
            print(self.offsets[2,2]+pos[2][2])
        except:
            print("back_right_lower error caught")
        #timeSleep(0.04)
    #if servo_name == "bl" or servo_name == "all":
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_left_hip    ,self.offsets[0,3]+pos[3][0])
            print(self.offsets[0,3]+pos[3][0])
        except:
            print("back_left_hip error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_left_upper  ,self.offsets[1,3]+pos[3][1])
            print(self.offsets[1,3]+pos[3][1])
        except:
            print("back_left_upper error caught")
        #timeSleep(0.04)
        try:
            self.Dingo.moveAbsAngle(self.Dingo.back_left_lower  ,self.offsets[2,3]+pos[3][2])
            print(self.offsets[2,3]+pos[3][2])
        except:
            print("back_left_lower error caught")
        #timeSleep(0.04)
        return pos

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    rospy.init_node("keyboard_input_listener")
    rate = rospy.Rate(30) # 30 Hz

    if os.getenv("DISPLY", default="-") != "-":
        rospy.logfatal("This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
        rospy.sleep(1)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    keyboard_listener = Keyboard()

    try:
        while not rospy.is_shutdown():
            keyboard_listener.publish_current_command()
            time.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard node interrupted.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard node stopped by user.")
    finally:
        rospy.loginfo("Exiting...")
        stop_listening()
        sys.exit(0)


if __name__ == "__main__":
    main()