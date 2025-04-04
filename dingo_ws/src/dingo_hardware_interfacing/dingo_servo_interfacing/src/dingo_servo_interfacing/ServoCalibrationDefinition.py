#!/usr/bin/env python3
from adafruit_servokit import ServoKit
# from leg_config import *
import numpy as np
import math as m
import time

class motor_config():
    def __init__(self):
        self.pwm_max = 2400 #1x time speed
        self.pwm_min = 370
        self.kitF = ServoKit(channels=16) #Defining a new set of servos uising the Adafruit ServoKit LIbrary
        self.kitB = ServoKit(channels=16, address=0x41) #Defininng a new set of servos uising the Adafruit ServoKit LIbrary
          
        #DefinING servo indices
        ## FRONT LEFT
        self.front_left_hip   = 10
        self.front_left_upper = 9
        self.front_left_lower = 8

        ## FRONT RIGHT
        self.front_right_hip   = 14
        self.front_right_upper = 13
        self.front_right_lower = 12

        ## BACK LEFT
        self.back_left_hip   = 6
        self.back_left_upper = 5
        self.back_left_lower = 4

        ## BACK RIGHT
        self.back_right_hip   = 2
        self.back_right_upper = 1
        self.back_right_lower = 0
        """ SERVO INDICES, CALIBRATION MULTIPLIERS AND OFFSETS
            #   ROW:    which joint of leg to control 0:hip, 1: upper leg, 2: lower leg
            #   COLUMN: which leg to control. 0: front-right, 1: front-left, 2: back-right, 3: back-left.

                #               0                  1                2               3
                #  0 [[front_right_hip  , front_left_hip  , back_right_hip  , back_left_hip  ]
                #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
                #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]] """

        self.pins = np.array([[14,10,2,6], 
                              [13,9,1,5], 
                              [12,8,0,4]])

        self.right_leg_servo_list = [self.front_right_upper,self.front_right_lower,self.back_right_upper,self.back_right_lower]
        self.left_leg_servos_list = [ self.front_left_upper, self.front_left_lower,self.back_left_upper,self.back_left_lower]
        self.hip_opposite_list = [self.front_right_hip,self.back_left_hip]
        self.hip_opposite_list2 = [self.front_left_hip,self.back_right_hip]
        
        # self.front_leg_servo_list = [self.front_right_upper,self.front_right_lower, self.front_left_upper, self.front_left_lower]
        # self.back_leg_servos_list = [self.back_left_upper,self.back_left_lower, self.back_right_upper,self.back_right_lower]
        # self.hip_opposite_list = [self.front_right_hip,self.back_left_hip]
        
        #applying calibration values to all servos
        self.create()

    def create(self):
        for i in range(16):
            self.kitF.servo[i].actuation_range = 180
            self.kitF.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)
            
            self.kitB.servo[i].actuation_range = 180
            self.kitB.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)
    def calibrate_servo(self,servo_number):
        cal = False
        while cal == False:  
            angle = input('InputServo angle: ')
            self.kit.servo[servo_number].angle = float(angle)
            response = input('Is the servo fully vertical? y/n')
            if response == 'y':
                cal = True
                calibration_angle = float( angle)
                print('The calibration angle for servo ',servo_number, ' is ', calibration_angle)
                return
    def moveAbsAngle(self,servo_number,angle):
        if angle < 0:
            angle = 0
        if angle > 180:
            angle = 180
        # Takes 180-angle so that the movement it the same as the right lef
        if servo_number in self.left_leg_servos_list:
            self.kitF.servo[servo_number].angle = 180 - angle
            time.sleep(0.03)
            self.kitB.servo[servo_number].angle = 180 - angle
            time.sleep(0.03)
            #print("1")
        elif servo_number in self.hip_opposite_list: #corrects hip angle such that higher numbers are angles of elevation. Higher hip values fo all lift up
            self.kitF.servo[servo_number].angle = 180-angle
            time.sleep(0.03)
            
            self.kitB.servo[servo_number].angle = 180-angle
            time.sleep(0.03)
            #print("2")
        elif servo_number in self.hip_opposite_list2:
            self.kitF.servo[servo_number].angle = angle
            time.sleep(0.03)
            self.kitB.servo[servo_number].angle = angle
            time.sleep(0.03)
            #print("3")
        else:
            self.kitF.servo[servo_number].angle = angle
            time.sleep(0.03)
            self.kitB.servo[servo_number].angle = angle
            time.sleep(0.03)
            #print("4")
    def relax_all_motors(self):
        """Relaxes desired servos so that they appear to be turned off. 

        Parameters
        ----------
        servo_list : 3x4 numpy array of 1's and zeros. Row = Actuator; Column = leg.
                    If a Given actuator is 0 is 1 it should be deactivated, if it is 0 is should be left on. 
        """

        for i in range(16):
            try:
                self.kitF.servo[i].angle = None
                self.kitB.servo[i].angle = None
            except Exception as e:
                print(f"Error occurred with servo {i}. Error message: {e}")
        


        #-------------------- ##CALIBRATRION ANGLES---------------------#
        #    TODO: put these angle in a refrencable file or matrix. Maybe use a text file that can be
        #          editied using the 'calibrate_servo' method.
        
        # ALl calibration angles are defined as the required angular input to the servo to achieve a configuration that is:
        #  - for hip servos, perfectly vertical. Increasing hip angles lifts the hip up
        #  - for Upper legs, facing direclty horizontal toward the back of the robot
        #  - for lower leg servos, facing directly down from the hip.

        # ## FRONT RIGHT
        # self.front_right_hip   = 113 degrees, lower number lifts up
        # self.front_right_upper = 29
        # self.front_right_lower = 24 

        # ## FRONT LEFT
        # self.front_left_hip   =  74  higher number lifts up
        # self.front_left_upper = 17 (all angles to be output are: 180-(theta + 6)  )
        # self.front_left_lower = 0 (all angles to be output are: 180-(theta + 7)  )

        # ## BACK RIGHT
        # self.back_right_hip   = 105 DEGREES, higher number lifts up
        # self.back_right_upper = 22
        # self.back_right_lower = 30

        # ## BACK LEFT
        # self.back_left_hip   = 83 (all angles to be output are: 180-(theta + 6)  )
        # self.back_left_upper = 9
        # self.back_left_lower = 5

