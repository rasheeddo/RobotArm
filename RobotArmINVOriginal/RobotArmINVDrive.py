# Test servo with Maestro 

import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import pygame
import numpy
import math

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

def RobotArmINV(qx,qy,qz,x6_rot,y6_rot,z6_rot):

    rad2deg = 180/math.pi
    deg2rad = math.pi/180
    pi = math.pi
    ########################## parameters ################################
    a1 = 0
    d1 = 0
    alp1 = pi/2

    a2 = 216
    d2 = 0
    alp2 = 0

    a3 = 74
    d3 = 0
    alp3 = pi/2

    a4 = 0
    d4 = 142
    alp4 = -pi/2

    a5 = 0
    d5 = 0
    alp5 = pi/2

    a6 = 0
    d6 = 200
    alp6 = 0

    '''
    qx = 200
    qy = 300
    qz = 150

    x6_rot = 0
    y6_rot = 0
    z6_rot = 0
    '''

    rr = x6_rot*deg2rad
    pp = y6_rot*deg2rad
    yy = z6_rot*deg2rad

    E11 = math.cos(yy)*math.cos(pp)
    E12 = math.cos(yy)*math.sin(pp)*math.sin(rr)-math.cos(rr)*math.sin(yy)
    E13 = math.sin(rr)*math.sin(yy)+math.cos(rr)*math.cos(yy)*math.sin(pp)

    E21 = math.cos(pp)*math.sin(yy)
    E22 = math.cos(rr)*math.cos(yy) + math.sin(rr)*math.sin(yy)*math.sin(pp)
    E23 = math.cos(rr)*math.sin(yy)*math.sin(pp)-math.cos(yy)*math.sin(rr)

    E31 = -math.sin(pp)
    E32 = math.cos(pp)*math.sin(rr)
    E33 = math.cos(rr)*math.cos(pp)

    TCP = numpy.matrix([[E11,E12,E13],[E21,E22,E23],[E31,E32,E33]])

    r = 180*deg2rad
    p = -90*deg2rad
    y = 90*deg2rad

    psi = y
    theta = p
    phi = r

    F11 = math.cos(psi)*math.cos(theta)
    F12 = math.cos(psi)*math.sin(phi)*math.sin(theta)-math.cos(phi)*math.sin(psi)
    F13 = math.sin(phi)*math.sin(psi)+math.cos(phi)*math.cos(psi)*math.sin(theta)

    F21 = math.cos(theta)*math.sin(psi)
    F22 = math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(psi)*math.sin(theta)
    F23 = math.cos(phi)*math.sin(psi)*math.sin(theta)-math.cos(psi)*math.sin(phi)

    F31 = -math.sin(theta)
    F32 = math.cos(theta)*math.sin(phi)
    F33 = math.cos(phi)*math.cos(theta)

    RPY = numpy.matrix([[F11,F12,F13],[F21,F22,F23],[F31,F32,F33]])

    RPY_TCP = RPY*TCP

    ux = RPY_TCP[0,0]
    uy = RPY_TCP[1,0]
    uz = RPY_TCP[2,0]

    vx = RPY_TCP[0,1]
    vy = RPY_TCP[1,1]
    vz = RPY_TCP[2,1]

    wx = RPY_TCP[0,2]
    wy = RPY_TCP[1,2]
    wz = RPY_TCP[2,2]

    px = qx - d6*wx
    py = qy - d6*wy
    pz = qz - d6*wz


    ################################ Find q1 #####################################
    q1 = math.atan(py/px)

    if ((q1 <= pi) and (q1 >= 0)):
        q1_1 = q1
        q1_2 = q1 + pi
    else:
        q1_1 = q1 + pi
        q1_2 = q1
        q1 = q1 + pi
    '''
    if q1_1 < 0.000001:
        q1_2 = 0

    if q1_2 < 0.000001:
        q1_2 = 0
    '''
    deg1 = [None]*2
    deg1 = numpy.array([q1_1, q1_2])*rad2deg



    #print(q1)

    ################################ Find q3 #####################################
    k1 = 2*a2*d4
    k2 = 2*a2*a3
    k3 = px**2 + py**2 + pz**2 - 2*px*a1*math.cos(q1) - 2*py*a1*math.sin(q1) + a1**2 - a2**2 - a3**2 - d4**2

    q3_1 = 2*math.atan( (k1+ math.sqrt(k1**2 + k2**2 - k3**2)) / (k3+k2) )
    q3_2 = 2*math.atan( (k1- math.sqrt(k1**2 + k2**2 - k3**2)) / (k3+k2) )

    '''
    if q3_1 < 0.000001:
        q3_1 = 0
    if q3_2 < 0.000001:
        q3_2 = 0
    '''
    q3 = [None]*2
    q3 = numpy.array([q3_1,q3_2])


    #print(q3)
    deg3 = [None]*2
    deg3 = numpy.array([q3[0],q3[1]])*rad2deg

    ################################ Find q2 #####################################

    #print("----------------")

    uu1_0 = a2 + a3*math.cos(q3[0])+d4*math.sin(q3[0])
    vv1_0 = -a3*math.sin(q3[0]) + d4*math.cos(q3[0])
    yy1_0 = px*math.cos(q1) + py*math.sin(q1) - a1
    uu2_0 = a3*math.sin(q3[0]) - d4*math.cos(q3[0])
    vv2_0 = a2 + a3*math.cos(q3[0]) + d4*math.sin(q3[0])
    yy2_0 = pz

    uu1_1 = a2 + a3*math.cos(q3[1])+d4*math.sin(q3[1])
    vv1_1 = -a3*math.sin(q3[1]) + d4*math.cos(q3[1])
    yy1_1 = px*math.cos(q1) + py*math.sin(q1) - a1
    uu2_1 = a3*math.sin(q3[1]) - d4*math.cos(q3[1])
    vv2_1 = a2 + a3*math.cos(q3[1]) + d4*math.sin(q3[1])
    yy2_1 = pz


    A0 = numpy.array([[uu1_0,vv1_0],[uu2_0,vv2_0]])
    B0 = numpy.array([yy1_0,yy2_0])
    X0 = numpy.linalg.solve(A0,B0)

    A1 = numpy.array([[uu1_1,vv1_1], [uu2_1,vv2_1]])
    B1 = numpy.array([yy1_1,yy2_1])
    X1 = numpy.linalg.solve(A1,B1)

    cq2_1 = X0[0]
    sq2_1 = X0[1]
    cq2_2 = X1[0]
    sq2_2 = X1[1]

    q2 = [None]*2
    q2[0] = math.atan2(sq2_1,cq2_1)
    q2[1] = math.atan2(sq2_2,cq2_2)
    '''
    if q2[0] < 0.000001:
        q2[0] = 0
    if q2[1] < 0.000001:
        q2[1] = 0
    '''
    deg2 = [None]*2
    deg2 = numpy.array([q2[0],q2[1]])*rad2deg

    #print(q2)
    #print(deg2)

    ################################ Find q5 #####################################

    r33 = [None]*2
    r33[0] = wx*math.cos(q1)*math.sin(q2[0]+q3[0]) + wy*math.sin(q1)*math.sin(q2[0]+q3[0]) - wz*math.cos(q2[0]+q3[0])
    r33[1] = wx*math.cos(q1)*math.sin(q2[1]+q3[1]) + wy*math.sin(q1)*math.sin(q2[1]+q3[1]) - wz*math.cos(q2[1]+q3[1])
    #print(r33)
    q5 = [None]*2
    q5[0] = math.acos(r33[0])
    q5[1] = math.acos(r33[1])
    '''
    if q5[0] < 0.000001:
        q5[0] = 0
    if q5[1] < 0.000001:
        q5[1] = 0
    '''
    deg5 = [None]*2
    deg5 = numpy.array([q5[0],q5[1]])*rad2deg

    #print("----------------")
    #print(q5)
    #print(deg5)

    ################################ Find q4 #####################################
    cq4 = [None]*2
    sq4 = [None]*2
    q4 = [None]*2
    deg4 = [None]*2

    if (abs(r33[1]) < 1):
        # Normal Condition
        cq4[0] = (  wx*math.cos(q1)*math.cos(q2[0]+q3[0]) + wy*math.sin(q1)*math.cos(q2[0]+q3[0]) + wz*math.sin(q2[0]+q3[0]) ) / math.sin(q5[0])
        cq4[1] = (  wx*math.cos(q1)*math.cos(q2[1]+q3[1]) + wy*math.sin(q1)*math.cos(q2[1]+q3[1]) + wz*math.sin(q2[1]+q3[1]) ) / math.sin(q5[1])
        sq4[0] = ( wx*math.sin(q1) - wy*math.cos(q1) )/ math.sin(q5[0])
        sq4[1] = ( wx*math.sin(q1) - wy*math.cos(q1) )/ math.sin(q5[1])

        q4[0] = math.atan2(sq4[0],cq4[0])
        q4[1] = math.atan2(sq4[1],cq4[1])
    elif abs(r33[1]) == 1:
        # When q5 = 0
        q4[0] = 0
        q4[1] = 0

    else:
        print("|r33| > 1 : cannotr physically arise")
    '''
    if q4[0] < 0.000001:
        q4[0] = 0
    if q4[1] < 0.000001:
        q4[1] = 0
    '''
    deg4 = numpy.array([q4[0],q4[1]])*rad2deg
    #print("-----------------")
    #print(q4)
    #print(deg4)

    ################################ Find q6 #####################################
    cq6 = [None]*2
    sq6 = [None]*2
    q6 = [None]*2
    deg6 = [None]*2

    if (abs(r33[1]) < 1):
        # Normal Condition
        cq6[0] = -(ux*math.cos(q1)*math.sin(q2[0]+q3[0]) + uy*math.sin(q1)*math.sin(q2[0]+q3[0]) - uz*math.cos(q2[0]+q3[0]) ) / math.sin(q5[0])
        cq6[1] = -(ux*math.cos(q1)*math.sin(q2[1]+q3[1]) + uy*math.sin(q1)*math.sin(q2[1]+q3[1]) - uz*math.cos(q2[1]+q3[1]) ) / math.sin(q5[1])
        sq6[0] = (vx*math.cos(q1)*math.sin(q2[0]+q3[0]) + vy*math.sin(q1)*math.sin(q2[0]+q3[0]) - vz*math.cos(q2[0]+q3[0]) ) / math.sin(q5[0])
        sq6[1] = (vx*math.cos(q1)*math.sin(q2[1]+q3[1]) + vy*math.sin(q1)*math.sin(q2[1]+q3[1]) - vz*math.cos(q2[1]+q3[1]) ) / math.sin(q5[1])

        q6[0] = math.atan2(sq6[0],cq6[0])
        q6[1] = math.atan2(sq6[1],cq6[1])
    elif abs(r33[1]) == 1:
        # When q5 = 0
        q6[0] = 0
        q6[1] = 0
    else:
        print("|r33| > 1 : cannotr physically arise")
    '''
    if q6[0] < 0.000001:
        q6[0] = 0
    if q6[1] < 0.000001:
        q6[1] = 0
    '''
    deg6 = numpy.array([q6[0],q6[1]])*rad2deg
    #print("-----------------")
    #print(q6)
    #print(deg6)

    DEG_INV = [None]*6
    DEG_INV = numpy.array([deg1[0],deg2[1],deg3[1],deg4[1],deg5[1],deg6[1]])
    ServoAng1 = DEG_INV[0]+90
    ServoAng2 = DEG_INV[1]+90
    ServoAng3 = DEG_INV[2]+180
    ServoAng4 = DEG_INV[3]+180
    ServoAng5 = DEG_INV[4]+180
    ServoAng6 = DEG_INV[5]+180
    ServoANG = [None]*6
    ServoANG = numpy.array([ServoAng1,ServoAng2,ServoAng3,ServoAng4,ServoAng5,ServoAng6])

    return ServoANG

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
    DelAng1 = abs(Deg1 - Last_Ang1)
    DelAng2 = abs(Deg2 - Last_Ang2)
    DelAng3 = abs(Deg3 - Last_Ang3)
    DelAng4 = abs(Deg4 - Last_Ang4)
    DelAng5 = abs(Deg5 - Last_Ang5)
    DelAng6 = abs(Deg6 - Last_Ang6)
    if (DelAng1 < 20) and (DelAng2 < 20) and (DelAng3 < 20) and (DelAng4 < 20) and (DelAng5 < 20) and (DelAng6 < 25):
        print("Robot is in sleep position")
        time.sleep(1)
        AwakenOK = True
        TorqueOn()
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
    
    else:
        print("WARNING!: Robot is not in sleep position")
        AwakenOK = False
        print("Delta Deg1: %f" %DelAng1)
        print("Delta Deg2: %f" %DelAng2)
        print("Delta Deg3: %f" %DelAng3)
        print("Delta Deg4: %f" %DelAng4)
        print("Delta Deg5: %f" %DelAng5)
        print("Delta Deg6: %f" %DelAng6)

        TorqueOff()

    

    return AwakenOK




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

WakeUp = RobotArmAwake()
if WakeUp:
    print("All Torque are ON")
    time.sleep(1)
    StandByPos()
    print("On the stand by position")
    time.sleep(2)
    waitForStart = True
    print("Press Start Button to Run!")
else:
    waitForStart = False
    startINV = False


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
Continue_Btn = Buttons[8] #Logiccool
GripOpen_Btn = Buttons[9] # Analog Left Push
GripClose_Btn = Buttons[10] #Analog Right Push

DegIncrement = 3

################################################## Jog Joint ###################################################


while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        startINV = True
        waitForChooseAxis = True

    time.sleep(0.1)


print("-----------------------------------------------------------")
print("-----------------------------------------------------------")
print("-------------------Inverse Run started!----------------------")
print("-----------------------------------------------------------")
print("-----------------------------------------------------------")
'''
print("Joint1        -->        A")
print("Joint2        -->        B")
print("Joint3        -->        X")
print("Joint4        -->        Y")
print("Joint5        -->        LB")
print("Joint6        -->        RB")
print("Gripper open  --> Analog Left Push")
print("Gripper close --> Analog Right Push")
'''

while startINV:

    print("Press X for X translation")
    print("Press Y for Y translation")
    print("Press B for Z translation")
    print("Press Back for exit")

    while waitForChooseAxis:
        Buttons = getButton()
        Z_Btn = Buttons[1] #B
        X_Btn = Buttons[2] #X
        Y_Btn = Buttons[3] #Y
        Back_Btn = Buttons[6] #Back

        if X_Btn == 1:
            X_Mode = True
            Y_Mode = False
            Z_Mode = False
            waitForChooseAxis = False
        
        elif Y_Btn == 1:
            X_Mode = False
            Y_Mode = True
            Z_Mode = False
            waitForChooseAxis = False
        
        elif Z_Btn == 1:
            X_Mode = False
            Y_Mode = False
            Z_Mode = True
            waitForChooseAxis = False

        if Back_Btn == 1:
            waitForChooseAxis = False
            startINV = False


    if X_Mode:

        XinitialValue = -250 #  Must not exceed -300
        XMaxValue = 250  #  Must not exceed 300
        x = XinitialValue
        y = 300
        z = 150
        X_Rot = 0
        Y_Rot = 0
        Z_Rot = 0
        StepIncrement = 5
        startINVX = True

        while startINVX:

            if x == XMaxValue:
                waitForRunAgain = True
                print("Press Start to run again!")
                print("Or Press Back to end program...")
                while waitForRunAgain:
                    Buttons = getButton()
                    Start_Btn = Buttons[7] #Start
                    Back_Btn = Buttons[6] #Back
                    if Start_Btn == 1:
                        x = XinitialValue
                        waitForRunAgain = False
                        StandByPos()
                        print("Robot is now ready to run again")
                        time.sleep(3)
                    elif Back_Btn == 1:
                        waitForRunAgain = False
                        startINVX = False
                        X_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                        print("Finished...")
                
                
            else:
                startINVX = True

                DriveAng = RobotArmINV(x,y,z,X_Rot,Y_Rot,Z_Rot)
                DriveANG1 = DriveAng[0]
                DriveANG2 = DriveAng[1]
                DriveANG3 = DriveAng[2]
                DriveANG4 = DriveAng[3]
                DriveANG5 = DriveAng[4]
                DriveANG6 = DriveAng[5]

                print("Deg1:%f" %DriveANG1)
                print("Deg2:%f" %DriveANG2)
                print("Deg3:%f" %DriveANG3)
                print("Deg4:%f" %DriveANG4)
                print("Deg5:%f" %DriveANG5)
                print("Deg6:%f" %DriveANG6)
                print("")

                RunServo(DriveANG1,DriveANG2,DriveANG3,DriveANG4,DriveANG5,DriveANG6)
                waitForNextStep = True

                '''
                if i == 0:
                    time.sleep(3)
                '''
                print("X: %f" %x)
                print("Press Logiccool Button to continue...")
                print("Press Back to Exit")
                print("---------------------------------------")

                while waitForNextStep:
                    Buttons = getButton()
                    Back_Btn = Buttons[6] #Back
                    Continue_Btn = Buttons[8] #Logiccool
                    if Continue_Btn == 1:
                        waitForNextStep = False
                    elif Back_Btn == 1:
                        waitForNextStep = False
                        startINVX = False
                        X_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                    else:
                        waitForNextStep = True

                x = x + StepIncrement

    if Y_Mode:

        YinitialValue = 250 #  Must not lower than 200
        YMaxValue = 450  #  Must not exceed 500
        x = 0
        y = YinitialValue
        z = 150
        X_Rot = 0
        Y_Rot = 0
        Z_Rot = 0
        StepIncrement = 5
        startINVY = True

        while startINVY:

            if y == YMaxValue:
                waitForRunAgain = True
                print("Press Start to run again!")
                print("Or Press Back to end program...")
                while waitForRunAgain:
                    Buttons = getButton()
                    Start_Btn = Buttons[7] #Start
                    Back_Btn = Buttons[6] #Back
                    if Start_Btn == 1:
                        y = YinitialValue
                        waitForRunAgain = False
                        StandByPos()
                        print("Robot is now ready to run again")
                        time.sleep(3)
                    elif Back_Btn == 1:
                        waitForRunAgain = False
                        startINVY = False
                        Y_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                        print("Finished...")
                
                
            else:
                startINVY = True

                DriveAng = RobotArmINV(x,y,z,X_Rot,Y_Rot,Z_Rot)
                DriveANG1 = DriveAng[0]
                DriveANG2 = DriveAng[1]
                DriveANG3 = DriveAng[2]
                DriveANG4 = DriveAng[3]
                DriveANG5 = DriveAng[4]
                DriveANG6 = DriveAng[5]

                print("Deg1:%f" %DriveANG1)
                print("Deg2:%f" %DriveANG2)
                print("Deg3:%f" %DriveANG3)
                print("Deg4:%f" %DriveANG4)
                print("Deg5:%f" %DriveANG5)
                print("Deg6:%f" %DriveANG6)
                print("")

                RunServo(DriveANG1,DriveANG2,DriveANG3,DriveANG4,DriveANG5,DriveANG6)
                waitForNextStep = True

                '''
                if i == 0:
                    time.sleep(3)
                '''
                print("Y: %f" %y)
                print("Press Logiccool Button to continue...")
                print("Press Back to Exit")
                print("---------------------------------------")

                while waitForNextStep:
                    Buttons = getButton()
                    Back_Btn = Buttons[6] #Back
                    Continue_Btn = Buttons[8] #Logiccool
                    if Continue_Btn == 1:
                        waitForNextStep = False
                    elif Back_Btn == 1:
                        waitForNextStep = False
                        startINVY = False
                        Y_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                    else:
                        waitForNextStep = True

                y = y + StepIncrement

    if Z_Mode:

        ZinitialValue = 300 #  Must not exceed 300
        ZMaxValue = 100  #  Must not exceed 100
        x = 0
        y = 300
        z = ZinitialValue
        X_Rot = 0
        Y_Rot = 0
        Z_Rot = 0
        StepIncrement = -5
        startINVZ = True

        while startINVZ:

            if z == ZMaxValue:
                waitForRunAgain = True
                print("Press Start to run again!")
                print("Or Press Back to end program...")
                while waitForRunAgain:
                    Buttons = getButton()
                    Start_Btn = Buttons[7] #Start
                    Back_Btn = Buttons[6] #Back
                    if Start_Btn == 1:
                        z = ZinitialValue
                        waitForRunAgain = False
                        StandByPos()
                        print("Robot is now ready to run again")
                        time.sleep(3)
                    elif Back_Btn == 1:
                        waitForRunAgain = False
                        startINVZ = False
                        Z_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                        print("Finished...")
                
                
            else:
                startINVZ = True

                DriveAng = RobotArmINV(x,y,z,X_Rot,Y_Rot,Z_Rot)
                DriveANG1 = DriveAng[0]
                DriveANG2 = DriveAng[1]
                DriveANG3 = DriveAng[2]
                DriveANG4 = DriveAng[3]
                DriveANG5 = DriveAng[4]
                DriveANG6 = DriveAng[5]

                print("Deg1:%f" %DriveANG1)
                print("Deg2:%f" %DriveANG2)
                print("Deg3:%f" %DriveANG3)
                print("Deg4:%f" %DriveANG4)
                print("Deg5:%f" %DriveANG5)
                print("Deg6:%f" %DriveANG6)
                print("")

                RunServo(DriveANG1,DriveANG2,DriveANG3,DriveANG4,DriveANG5,DriveANG6)
                waitForNextStep = True

                '''
                if i == 0:
                    time.sleep(3)
                '''
                print("Z: %f" %y)
                print("Press Logiccool Button to continue...")
                print("Press Back to Exit")
                print("---------------------------------------")

                while waitForNextStep:
                    Buttons = getButton()
                    Back_Btn = Buttons[6] #Back
                    Continue_Btn = Buttons[8] #Logiccool
                    if Continue_Btn == 1:
                        waitForNextStep = False
                    elif Back_Btn == 1:
                        waitForNextStep = False
                        startINVZ = False
                        Z_Mode = False
                        waitForChooseAxis = True
                        time.sleep(1)
                    else:
                        waitForNextStep = True

                z = z + StepIncrement


    




if WakeUp:
    waitForShutDown = True
    print("Press Back Button to shutdown the robot...")
    while waitForShutDown:
        Buttons = getButton()
        Back_Btn = Buttons[6] #Back
        if Back_Btn == 1:
            waitForShutDown = False
        else:
            waitForShutDown = True

    time.sleep(1)
    StandByPos()
    time.sleep(1)
    RobotArmGoHome()
    time.sleep(1)
    TorqueOff()
    WakeUp = False


if __name__ == '__main__':
    print("yo yo what's up")