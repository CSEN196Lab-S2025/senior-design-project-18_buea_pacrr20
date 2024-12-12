#!/usr/bin/env python3
# coding: utf-8
import sys
import numpy as np
import math as m
import time
from timer import timer

from ServoCalibrationDefinition import motor_config

# FIRST define a new motor class
Dingo  = motor_config()

offsets = np.array(
                    # [[70, 107, 115, 64],
                    # [35, 10, 15, 22],
                    # [16, 27, 35, 14]])
                    [[64, 109, 115, 77],
                     [ 15,   7,  11, 19],
                     [ 2,   -5,  -8,  -6]])

def timeSleep(duration):
    with timer() as t:
        while True:
            if t.elapse >= duration:
                break

def move_legs(pos,current_pos):
    increments = np.subtract(pos,current_pos)/3
    while not np.array_equal(current_pos,pos):
        current_pos = np.add(current_pos,increments)
        
        #for leg in range(0,len(increments)): # 4 legs
        #    for angle in range(0,len(increments[leg])): # 3 joints
        #        if (increments[leg][angle] >= 0):
        #            current_pos[leg][angle]=min(current_pos[leg][angle],pos[leg][angle])
        #        else:
        #            current_pos[leg][angle]=max(current_pos[leg][angle],pos[leg][angle])

    #if servo_name == "fr" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.front_right_hip  ,offsets[0,0]+current_pos[0][0])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.front_right_upper,offsets[1,0]+current_pos[0][1])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.front_right_lower,offsets[2,0]+current_pos[0][2])
        timeSleep(0.04)
    #if servo_name == "fl" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.front_left_hip   ,offsets[0,1]+current_pos[1][0])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.front_left_upper ,offsets[1,1]+current_pos[1][1])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.front_left_lower ,offsets[2,1]+current_pos[1][2])
        timeSleep(0.04)
    #if servo_name == "br" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.back_right_hip   ,offsets[0,2]+current_pos[2][0])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.back_right_upper ,offsets[1,2]+current_pos[2][1])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.back_right_lower ,offsets[2,2]+current_pos[2][2])
        timeSleep(0.04)
    #if servo_name == "bl" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.back_left_hip    ,offsets[0,3]+current_pos[3][0])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.back_left_upper  ,offsets[1,3]+current_pos[3][1])
        timeSleep(0.04)
        Dingo.moveAbsAngle(Dingo.back_left_lower  ,offsets[2,3]+current_pos[3][2])
        timeSleep(0.04)
    return pos



'''    HOW TO CALIBRATE THE MOTORS
This is how the robot should look at the calibbration position of [0,0,90]
                            LINKAGE
                          /‾‾‾‾‾‾‾\------------------- q
                         /   _______                   |
                        |   |    o__|___UPPER LEG______/   <---- UPPER LEG AT 0° POINTS HORIZONTALLY BACKWARD
   LOWER LEG SERVO -->  |___|__o    |‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾/
      AT 90° POINTS         |       |                /
   HORIZONTALLY FORWARD      ‾‾‾‾‾‾‾                /
                            SERVO HUB              / LOWER LEG
                                                  /
                                                 O

CALIBRATION PROCESS
1. Mount servo hubs (without legs) on hip servos at approximately 90 degrees (middle of the servo's range)
2. Ensure all motors are powered and run this script with:
            pos = calibration_pos
                                                and,
            offsets = np.array(
                    [[90, 90, 90, 90],
                     [0 , 0 , 0 , 0 ],
                     [0 , 0 , 0 , 0 ]])
3. Mount upper leg and lower leg servo horn **such that a positive calibration angle will achieve the desired position**.
    So, upper leg should be slightly angled up toward the back of the robot and lower leg servo horn
    should be slightly angled down from the forward horizontal
4. Run this script repeatedly and adjust calibration offsets until the deesired position is reached.
    It is suggested to calibrate hips first.

    HIP   servos: positive angles rotate the hip up
    UPPER servos: positive angles rotate clockwise for left and anticlockwise for right (down on diagram)
    LOWER servos: positive angles rotate clockwise for left and anticlockwise for right (up on diagram)
5. Once calibration offsets have all been found, copy values of "offsets" array to the hardware interface
    and replace values of "self.physical_calibration_offsets"

'''

#-------- MOVING CALIBRATED LEGS TO THE HOME POSITION -------- #
# ## Home position values:
calibration_pos = [0,0,90] # [hip_servo angle, upper leg servo angle,lower leg servo angle]


# These three positions are presets for the robot standing in a low, medium and high stance.
# Used for testing only
low = [0,30,120]
lowmid = [0,35,118]
mid = [0,40,115]
midhigh = [0,45,112]
high = [0,50,110]

position_dict = {
    "cal": calibration_pos,
    "low": low,
    "lm": lowmid,
    "mid": mid,
    "mh": midhigh,
    "high": high
}

servo_dict = {
    "fr",
    "fl",
    "br",
    "bl",
    "all",
    "relax"
}

#  CHOOSE POSITION:
# pos = calibration_pos

#servo_move = ""
#if len(sys.argv) > 1 and sys.argv[1] in position_dict:
#    servo_move = sys.argv[2]
#    pos = position_dict[servo_move]
#fr,fl,br,bl?
if len(sys.argv) > 0:
    f = open("pushups.txt", "r")
    lines = f.readlines()
    positions=[]
    current_pos = [[0,42,100],[0,42,100],[0,42,100],[0,42,100]]

    for i in range(0,len(lines)):
        lines[i]=lines[i].strip()
        legs=lines[i].split('],[')
        for k in range(0,len(legs)):
            legs[k]=legs[k].strip('[]\n')
            legs[k]=np.fromstring(legs[k],dtype=int,sep=',')
        #print(legs)
        positions.append(legs)
    #print(positions)
    # for each position in the text file
    for pos in positions:
        # each leg has 3 servo angles
        #for leg in pos:
        #    if(leg[0] >= 15):
        #        leg[0]=15
        #    elif(leg[0] <= -25):
        #        leg[0]=-25
        #    if(leg[1] >= 70):
        #        leg[1]=70
        #    elif(leg[1] <= 0):
        #        leg[1]=0
        #    if(leg[2] >= 100):
        #        leg[2]=100
        #    elif(leg[2] <= 70):
        #        leg[2]=70
        #servo_move = np.array2string(pos)
        #print(pos,current_pos)
        current_pos=move_legs(pos,current_pos)
        #print(current_pos)
        timeSleep(0.16)

else:
    pos = calibration_pos

""" SERVO INDICES, CALIBRATION MULTIPLIERS AND OFFSETS
            #   ROW:    which joint of leg to control 0:hip, 1: upper leg, 2: lower leg
            #   COLUMN: which leg to control. 0: front-right, 1: front-left, 2: back-right, 3: back-left.

                #               0                  1                2               3
                #  0 [[front_right_hip  , front_left_hip  , back_right_hip  , back_left_hip  ]
                #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
                #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]] """


#servo_name = ""
#if len(sys.argv) > 1 and sys.argv[1] in servo_dict:
    #servo_name = sys.argv[1]

    #if servo_name != "relax":
    #    print('DINGO: Motor ' + servo_name + ' moved to ' + servo_move + '.\n')
    #else:
    #    print('DINGO: Motors Relaxed.\n')



#if servo_name == "relax":
#    Dingo.relax_all_motors()

# NOTE: It may be convenient to comment out some of the lines below to test individual motos/ legs

# FRONT RIGHT LEG
# Dingo.moveAbsAngle(Dingo.front_right_hip  ,offsets[0,0]+pos[0])
# Dingo.moveAbsAngle(Dingo.front_right_upper,offsets[1,0]+pos[1])
# Dingo.moveAbsAngle(Dingo.front_right_lower,offsets[2,0]+pos[2])
# FRONT LEFT LEG
# Dingo.moveAbsAngle(Dingo.front_left_hip   ,offsets[0,1]+pos[0])
# Dingo.moveAbsAngle(Dingo.front_left_upper ,offsets[1,1]+pos[1])
# Dingo.moveAbsAngle(Dingo.front_left_lower ,offsets[2,1]+pos[2])
# BACK RIGHT LEG
# Dingo.moveAbsAngle(Dingo.back_right_hip   ,offsets[0,2]+pos[0])
# Dingo.moveAbsAngle(Dingo.back_right_upper ,offsets[1,2]+pos[1])
# Dingo.moveAbsAngle(Dingo.back_right_lower ,offsets[2,2]+pos[2])
# BACK LEFT LEG
# Dingo.moveAbsAngle(Dingo.back_left_hip    ,offsets[0,3]+pos[0])
# Dingo.moveAbsAngle(Dingo.back_left_upper  ,offsets[1,3]+pos[1])
# Dingo.moveAbsAngle(Dingo.back_left_lower  ,offsets[2,3]+pos[2])


