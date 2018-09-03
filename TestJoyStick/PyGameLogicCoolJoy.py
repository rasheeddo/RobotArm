
import time
import pygame
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

def map(x):
    result = (x - 1)*(8000 - (-8000)) / (-1 - (1)) + (-8000)
    if result < 4000: return 4000
    if result > 8000: return 8000
    else: return result

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

def test():
    first = time.time()
    while True:
        
        Axes = getAxis()
        Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
        Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1
        Ax2 = Axes[2]       # LT unpressed = -1, LT pressed = 1
        Ax3 = Axes[3]       #Analog right push down = +1, Analog right push up = -1
        Ax4 = Axes[4]       #Analog right push right = +1, Analog right push left = -1
        Ax5 = Axes[5]       # RT unpressed = -1, RT pressed = 1
        '''
        des_throttle = rc_map(des_attitude[0],-1,1,-10,10)
        des_pitch = rc_map(des_attitude[1],1,-1,-45,45)
        des_roll = rc_map(des_attitude[2],-1,1,-45,45)
        des_yaw = rc_map(des_attitude[3],-1,1,-45,45)
        des_unknown1 = rc_map(des_attitude[4],-1,1,-45,45)
        des_unknown2 = rc_map(des_attitude[5],-1,1,-45,45)
        '''

        Hats = getHat()
        H0 = Hats # H0[0] = -1 pressed LeftDir
                  # H0[0] = 1 pressed RightDir 
                  # H0[0] = 0 no preess
                  # H0[1] = -1 pressed DownDir
                  # H0[1] = 1 pressed UpDir
                  # H0[1] = 0 no press
       
    
        Buttons = getButton()
        B0 = Buttons[0] #A
        B1 = Buttons[1] #B
        B2 = Buttons[2] #X
        B3 = Buttons[3] #Y
        B4 = Buttons[4] #LB
        B5 = Buttons[5] #RB
        B6 = Buttons[6] #Back
        B7 = Buttons[7] #Start
        B8 = Buttons[8] #Logiccool
        B9 = Buttons[9] # Analog Left Push
        B10 = Buttons[10] #Analog Right Push

        '''
        print("B0: %d" %B0)
        print("B1: %d" %B1)
        print("B2: %d" %B2)
        print("B3: %d" %B3)
        print("B4: %d" %B4)
        print("B5: %d" %B5)
        print("B6: %d" %B6)
        print("B7: %d" %B7)
        print("B8: %d" %B8)
        print("B9: %d" %B9)
        print("B10: %d" %B10)
        print("---------------------------")
       
        print("Ax0: %f" %Ax0)
        print("Ax1: %f" %Ax1)
        print("Ax2: %f" %Ax2)
        print("Ax3: %f" %Ax3)
        print("Ax4: %f" %Ax4)
        print("Ax5: %f" %Ax5)
        '''
        print("H0[0]: %d" %H0[0])
        print("H0[1]: %d" %H0[1])
        
        print("---------------------------")
        time.sleep(0.5)

        


test()
