# Test servo with Maestro 

import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import pygame

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
#print 'Initialized Joystick : %s' % j.get_name()
"""
Returns a vector of the following form:
[LThumbstickX, LThumbstickY, Unknown Coupled Axis???, 
RThumbstickX, RThumbstickY, 
Button 1/X, Button 2/A, Button 3/B, Button 4/Y, 
Left Bumper, Right Bumper, Left Trigger, Right Triller,
Select, Start, Left Thumb Press, Right Thumb Press]
Note:
No D-Pad.
Triggers are switches, not variable. 
Your controller may be different
"""

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result

def getButton():
    #Read input from the two joysticks
    pygame.event.pump()
    #unknown1 = j.get_axis(0)
    #unknown2 = j.get_axis(1)
    #throttle = j.get_axis(2)
    #roll = j.get_axis(4)
    #pitch = j.get_axis(3)
    #yaw = j.get_axis(5)
    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]
    
    return joy_button

def getAxis():
    #Read input from the two joysticks
    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(4)
    axis4 = j.get_axis(3)
    axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
    return joy_axis

def getHat():
    pygame.event.pump()
    hat0 = j.get_hat(0)
    
    joy_hat = hat0
    return joy_hat


def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def RunServo(inputDeg1,inputDeg2,inputDeg3,inputDeg4,inputDeg5,inputDeg6):

    pos1 = inputDeg1
    pos2 = inputDeg2
    pos3 = inputDeg3
    pos4 = inputDeg4
    pos5 = inputDeg5
    pos6 = inputDeg6

    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)
    servo_com4 = map(pos4,0.0,360.0,0.0,4095.0)
    servo_com5 = map(pos5,0.0,360.0,0.0,4095.0)
    servo_com6 = map(pos6,0.0,360.0,0.0,4095.0)

    dxl1_goal_position = int(servo_com1)
    dxl2_goal_position = int(servo_com2)
    dxl3_goal_position = int(servo_com3)
    dxl4_goal_position = int(servo_com4)
    dxl5_goal_position = int(servo_com5)
    dxl6_goal_position = int(servo_com6)

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    if dxl_comm_result3 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
    elif dxl_error3 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error3))

    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    if dxl_comm_result4 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result4))
    elif dxl_error4 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error4))

    dxl_comm_result5, dxl_error5 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
    if dxl_comm_result5 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result5))
    elif dxl_error5 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error5))

    dxl_comm_result6, dxl_error6 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
    if dxl_comm_result6 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result6))
    elif dxl_error6 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error6))

def ReadAngle():
    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position4, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position5, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position6, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PRESENT_POSITION)

    pre_pos1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
    pre_pos2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
    pre_pos3 = map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)
    pre_pos4 = map(dxl_present_position4, 0.0, 4095.0, 0.0, 360.0)
    pre_pos5 = map(dxl_present_position5, 0.0, 4095.0, 0.0, 360.0)
    pre_pos6 = map(dxl_present_position6, 0.0, 4095.0, 0.0, 360.0)

    return pre_pos1,pre_pos2,pre_pos3,pre_pos4,pre_pos5,pre_pos6

def RunServo1(inputDeg1):
    pos1 = inputDeg1
    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    dxl1_goal_position = int(servo_com1)
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

def RunServo2(inputDeg2):
    pos2 = inputDeg2
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    dxl2_goal_position = int(servo_com2)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

def RunServo3(inputDeg3):
    pos3 = inputDeg3
    servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)
    dxl3_goal_position = int(servo_com3)
    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    if dxl_comm_result3 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
    elif dxl_error3 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error3))

def RunServo4(inputDeg4):
    pos4 = inputDeg4
    servo_com4 = map(pos4,0.0,360.0,0.0,4095.0)
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    if dxl_comm_result4 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result4))
    elif dxl_error4 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error4))

def RunServo5(inputDeg5):
    pos5 = inputDeg5
    servo_com5 = map(pos5,0.0,360.0,0.0,4095.0)
    dxl5_goal_position = int(servo_com5)
    dxl_comm_result5, dxl_error5 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
    if dxl_comm_result5 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result5))
    elif dxl_error5 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error5))

def RunServo6(inputDeg6):
    pos6 = inputDeg6
    servo_com6 = map(pos6,0.0,360.0,0.0,4095.0)
    dxl6_goal_position = int(servo_com6)
    dxl_comm_result6, dxl_error6 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, dxl6_goal_position)
    if dxl_comm_result6 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result6))
    elif dxl_error6 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error6))

def TorqueOn():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

def TorqueOff():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

def GripOpen():
    dxl7_goal_position = 1700
    dxl_comm_result7, dxl_error7 = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_GOAL_POSITION, dxl7_goal_position)

def GripClose():
    dxl7_goal_position = 2780
    dxl_comm_result7, dxl_error7 = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_GOAL_POSITION, dxl7_goal_position)


def StandByPos():
    pos1 = 180
    pos2 = 180
    pos3 = 180
    pos4 = 180
    pos5 = 180
    pos6 = 180

    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)
    servo_com4 = map(pos4,0.0,360.0,0.0,4095.0)
    servo_com5 = map(pos5,0.0,360.0,0.0,4095.0)
    servo_com6 = map(pos6,0.0,360.0,0.0,4095.0)

    dxl1_goal_position = int(servo_com1)
    dxl2_goal_position = int(servo_com2)
    dxl3_goal_position = int(servo_com3)
    dxl4_goal_position = int(servo_com4)
    dxl5_goal_position = int(servo_com5)
    dxl6_goal_position = int(servo_com6)

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    dxl_comm_result5, dxl_error5 = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_goal_position)
    dxl_comm_result6, dxl_error6 = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_GOAL_POSITION, dxl6_goal_position)

def RobotArmGoHome():
    GripOpen()
    #### Point 0 ####
    Mem_Ang1 = 271.296703
    Mem_Ang2 = 194.021978
    Mem_Ang3 = 157.714286
    Mem_Ang4 = 182.769231
    Mem_Ang5 = 176.000000
    Mem_Ang6 = 181.186813
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

    #### Point 1 ####
    Mem_Ang1 = 278.417582
    Mem_Ang2 = 203.956044
    Mem_Ang3 = 157.978022
    Mem_Ang4 = 181.714286
    Mem_Ang5 = 123.164835
    Mem_Ang6 = 184.263736
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

    #### Point 2 ####
    Mem_Ang1 = 278.593407
    Mem_Ang2 = 220.131868
    Mem_Ang3 = 149.714286
    Mem_Ang4 = 182.857143
    Mem_Ang5 = 102.769231
    Mem_Ang6 = 183.560440
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

    #### Point 3 ####
    Mem_Ang1 = 283.956044
    Mem_Ang2 = 207.472527
    Mem_Ang3 = 116.307692
    Mem_Ang4 = 188.307692
    Mem_Ang5 = 132.747253
    Mem_Ang6 = 187.516484
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

    #### Point 4 ####
    Mem_Ang1 = 282.285714
    Mem_Ang2 = 188.483516
    Mem_Ang3 = 107.164835
    Mem_Ang4 = 194.021978
    Mem_Ang5 = 148.043956
    Mem_Ang6 = 184.087912
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

    #### Point 5 ####
    Mem_Ang1 = 282.637363
    Mem_Ang2 = 185.846154
    Mem_Ang3 = 106.109890
    Mem_Ang4 = 197.186813
    Mem_Ang5 = 152.879121
    Mem_Ang6 = 184.087912
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(1)

def RobotArmAwake():
    # Last Home Position #
    Last_Ang1 = 282.637363
    Last_Ang2 = 185.846154
    Last_Ang3 = 106.109890
    Last_Ang4 = 197.186813
    Last_Ang5 = 152.879121
    Last_Ang6 = 184.087912
    InitAng = ReadAngle()
    Deg1 = InitAng[0]
    Deg2 = InitAng[1]
    Deg3 = InitAng[2]
    Deg4 = InitAng[3]
    Deg5 = InitAng[4]
    Deg6 = InitAng[5]
    if (abs(Deg1 - Last_Ang1) < 10) and (abs(Deg2 - Last_Ang2) < 10) and (abs(Deg3 - Last_Ang3) < 10) and (abs(Deg4 - Last_Ang4) < 10) and (abs(Deg5 - Last_Ang5) < 10) and (abs(Deg6 - Last_Ang6) < 25):
        print("Robot is in sleep position")
        time.sleep(1)
        AwakenOK = True
        TorqueOn()
    else:
        print("WARNING!: Robot is not in sleep position")
        AwakenOK = False
        TorqueOff()

    GripOpen()
    #### Point 0 ####
    Mem_Ang1 = 277.802198
    Mem_Ang2 = 194.021978
    Mem_Ang3 = 110.329670
    Mem_Ang4 = 190.593407
    Mem_Ang5 = 142.593407
    Mem_Ang6 = 177.934066
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 1 ####
    Mem_Ang1 = 273.670330
    Mem_Ang2 = 199.384615
    Mem_Ang3 = 125.802198
    Mem_Ang4 = 186.901099
    Mem_Ang5 = 119.912088
    Mem_Ang6 = 177.934066
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 2 ####
    Mem_Ang1 = 271.560440
    Mem_Ang2 = 201.670330
    Mem_Ang3 = 152.351648
    Mem_Ang4 = 184.967033
    Mem_Ang5 = 96.351648
    Mem_Ang6 = 175.296703
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 3 ####
    Mem_Ang1 = 270.505495
    Mem_Ang2 = 213.890110
    Mem_Ang3 = 169.934066
    Mem_Ang4 = 184.439560
    Mem_Ang5 = 86.681319
    Mem_Ang6 = 174.153846
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 4 ####
    Mem_Ang1 = 272.175824
    Mem_Ang2 = 219.164835
    Mem_Ang3 = 155.604396
    Mem_Ang4 = 182.593407
    Mem_Ang5 = 180.219780
    Mem_Ang6 = 183.120879
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 5 ####
    Mem_Ang1 = 224.351648
    Mem_Ang2 = 175.560440
    Mem_Ang3 = 224.967033
    Mem_Ang4 = 173.450549
    Mem_Ang5 = 233.318681
    Mem_Ang6 = 181.098901
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 6 ####
    Mem_Ang1 = 180.659341
    Mem_Ang2 = 182.153846
    Mem_Ang3 = 186.373626
    Mem_Ang4 = 171.164835
    Mem_Ang5 = 222.857143
    Mem_Ang6 = 177.230769
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(0.5)

    #### Point 7 ####
    Mem_Ang1 = 177.494505
    Mem_Ang2 = 207.120879
    Mem_Ang3 = 111.208791
    Mem_Ang4 = 174.065934
    Mem_Ang5 = 222.857143
    Mem_Ang6 = 185.582418
    RunServo(Mem_Ang1,Mem_Ang2,Mem_Ang3,Mem_Ang4,Mem_Ang5,Mem_Ang6)
    time.sleep(2)



####################################################### Set Servo Configuration #############################################################
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
DXL4_ID                      = 4
DXL5_ID                      = 5
DXL6_ID                      = 6
DXL7_ID                      = 7

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# When need to change mode just remove block comment out
# Disable Dynamixel Torque 1
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 1 has been successfully connected")
# Disable Dynamixel Torque 2   
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 2 has been successfully connected")
# Disable Dynamixel Torque 3
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 3 has been successfully connected")
# Disable Dynamixel Torque 4
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 4 has been successfully connected")
# Disable Dynamixel Torque 5   
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 5 has been successfully connected")
# Disable Dynamixel Torque 6
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 6 has been successfully connected")

    # Check Operating Mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL)


present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
if present_mode == 0:
    # Current (Torque) Control Mode
    print("Now Operating Mode is Torque Control")
elif present_mode == 3:
    # Position Control Mode
    print("Now Operating Mode is Position Control")
elif present_mode == 5:
    # Current-based Position Control Mode
    print("Now Operating Mode is Current-based Position Control")
else:
    print("In other Mode that didn't set!")

'''
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Torque is enable")
'''
######################### Set Velocity / Acceleration Profile  ##############################
set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
set_V_Limit = 350       # 350 Default                  [0.229RPM]

set_A_PRFL = 15      # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL = 30      # between 0 ~ set_V_Limit      [0.229RPM]

set_A_PRFL_Gripper = 70      # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL_Gripper = 120      # between 0 ~ set_V_Limit      [0.229RPM]

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL_Gripper))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL_Gripper))

acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)
#print("Initial Position: %f" %start_pos)
#print("Final Position: %f" %final_pos)
#print("Travel time: %d" %t3)
print("V PRFL: %f" %set_V_PRFL)
print("A PRFL: %f" %set_A_PRFL)
print("Acceleration Limited: %d" %acceleration_limit)
print("Velocity Limited: %d" %velocity_limit)
print("--------------------------------")

######################### Set PID Gain Position Loop  ##############################
set_P_Gain = 1000    #800 default
set_I_Gain = 20     #0 default
set_D_Gain = 1000   #4700 default

set_P_Gain_Gripper = 1000
set_I_Gain_Gripper = 30
set_D_Gain_Gripper = 2000

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain_Gripper)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain_Gripper)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain_Gripper)
print("PID's Gain are set")

position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

position_D_gain_Gripper, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain_Gripper, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain_Gripper, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_POSITION_P_GAIN)

print("Robot PID Parameters")
print("Position P Gain: %d" %position_P_gain)
print("Position I Gain: %d" %position_I_gain)
print("Position D Gain: %d" %position_D_gain)
print("--------------------------------")
print("Gripper PID Parameters")
print("Gripper P Gain: %d" %position_P_gain_Gripper)
print("Gripper I Gain: %d" %position_I_gain_Gripper)
print("Gripper D Gain: %d" %position_D_gain_Gripper)
print("--------------------------------")
######################### Set Goal Current  ##############################
SetCur = 60
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Goal Current is set")
print("--------------------------------")

print("Ready to go....")
time.sleep(2)
################################################################################################################################
print("---------------------------------------------------")
print("---------Press start to wake the robot up----------")
print("---------------------------------------------------")
waitForAwake = True
while waitForAwake:
    Buttons = getButton()
    Start_Btn = Buttons[7] #Start
    if Start_Btn == 1:
        waitForAwake = False

    time.sleep(0.1)

RobotArmAwake()
print("All Torque are ON")
time.sleep(1)
StandByPos()
print("On the stand by position")
time.sleep(2)

################################################## Get Joy Stick ##############################################

Hats = getHat()
JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 

Buttons = getButton()
J1_Btn = Buttons[0] #A
J2_Btn = Buttons[1] #B
J3_Btn = Buttons[2] #X
J4_Btn = Buttons[3] #Y
J5_Btn = Buttons[4] #LB
J6_Btn = Buttons[5] #RB
Back_Btn = Buttons[6] #Back
Start_Btn = Buttons[7] #Start
StandBy_Btn = Buttons[8] #Logiccool
GripOpen_Btn = Buttons[9] # Analog Left Push
GripClose_Btn = Buttons[10] #Analog Right Push

DegIncrement = 3

################################################## Jog Joint ###################################################
waitForStart = True
print("Press Start Button to JogJoint!")

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startJog = True

    time.sleep(0.1)


print("-----------------------------------------------------------")
print("-----------------------------------------------------------")
print("-------------------Jog Joint started!----------------------")
print("-----------------------------------------------------------")
print("-----------------------------------------------------------")
print("Joint1        -->        A")
print("Joint2        -->        B")
print("Joint3        -->        X")
print("Joint4        -->        Y")
print("Joint5        -->        LB")
print("Joint6        -->        RB")
print("Gripper open  --> Analog Left Push")
print("Gripper close --> Analog Right Push")

while startJog:
    Buttons = getButton()
    J1_Btn = Buttons[0] #A
    J2_Btn = Buttons[1] #B
    J3_Btn = Buttons[2] #X
    J4_Btn = Buttons[3] #Y
    J5_Btn = Buttons[4] #LB
    J6_Btn = Buttons[5] #RB
    Back_Btn = Buttons[6] #Back
    Start_Btn = Buttons[7] #Start
    StandBy_Btn = Buttons[8] #Logiccool
    GripOpen_Btn = Buttons[9] # Analog Left Push
    GripClose_Btn = Buttons[10] #Analog Right Push

    ############ Jog Joint 1 ############
    while J1_Btn == 1:

        Buttons = getButton()
        J1_Btn = Buttons[0] #A

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg1 = ReadDeg[0]

        if JogDir == -1:
            DriveAngle = ReadDeg1 + DegIncrement
            RunServo1(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == 1:
            DriveAngle = ReadDeg1 - DegIncrement
            RunServo1(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 2 ############
    while J2_Btn == 1:

        Buttons = getButton()
        J2_Btn = Buttons[1] #B

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg2 = ReadDeg[1]

        if JogDir == -1:
            DriveAngle = ReadDeg2 + DegIncrement
            RunServo2(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == 1:
            DriveAngle = ReadDeg2 - DegIncrement
            RunServo2(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 3 ############
    while J3_Btn == 1:

        Buttons = getButton()
        J3_Btn = Buttons[2] #X

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg3 = ReadDeg[2]

        if JogDir == -1:
            DriveAngle = ReadDeg3 + DegIncrement
            RunServo3(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == 1:
            DriveAngle = ReadDeg3 - DegIncrement
            RunServo3(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 4 ############
    while J4_Btn == 1:

        Buttons = getButton()
        J4_Btn = Buttons[3] #Y

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg4 = ReadDeg[3]

        if JogDir == 1:
            DriveAngle = ReadDeg4 + DegIncrement
            RunServo4(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == -1:
            DriveAngle = ReadDeg4 - DegIncrement
            RunServo4(DriveAngle)
            #time.sleep(0.01)

    ############ Jog Joint 5 ############
    while J5_Btn == 1:

        Buttons = getButton()
        J5_Btn = Buttons[4] #LB

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg5 = ReadDeg[4]

        if JogDir == -1:
            DriveAngle = ReadDeg5 + DegIncrement
            RunServo5(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == 1:
            DriveAngle = ReadDeg5 - DegIncrement
            RunServo5(DriveAngle)
            #time.sleep(0.01)


    ############ Jog Joint 6 ############
    while J6_Btn == 1:

        Buttons = getButton()
        J6_Btn = Buttons[5] #RB

        Hats = getHat()
        JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 
        ReadDeg = ReadAngle()
        ReadDeg6 = ReadDeg[5]

        if JogDir == 1:
            DriveAngle = ReadDeg6 + DegIncrement
            RunServo6(DriveAngle)
            #time.sleep(0.01)
        elif JogDir == -1:
            DriveAngle = ReadDeg6 - DegIncrement
            RunServo6(DriveAngle)
            #time.sleep(0.01)
    if GripOpen_Btn == 1:
        GripOpen()

    if GripClose_Btn == 1:
        GripClose()

    if StandBy_Btn == 1:
        print("Robot is back to Stand by position...")
        time.sleep(1)
        StandByPos()

    if Back_Btn == 1:
        print("Robot is shutting down...")
        time.sleep(1)
        StandByPos()
        startJog = False

    time.sleep(0.1)

RobotArmGoHome()
print("--------------------------------------------------")
print("--------------------------------------------------")
print("-------------------Shutdown-----------------------")
print("--------------------------------------------------")
print("--------------------------------------------------")
time.sleep(1)
TorqueOff()
