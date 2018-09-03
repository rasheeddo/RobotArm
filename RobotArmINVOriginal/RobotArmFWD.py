
import time
import numpy
#import matplotlib.pyplot as plt
import math

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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

qx = 200
qy = 300
qz = 150


x6_rot = 0
y6_rot = 0
z6_rot = 0

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

print("****INPUT****")
print("qx: %f" %qx)
print("qy: %f" %qy)
print("qz: %f" %qz)
print("x6_rot: %f" %x6_rot)
print("y6_rot: %f" %y6_rot)
print("z6_rot: %f" %z6_rot)
print("roll: %f" %(r*rad2deg))
print("ptich: %f" %(p*rad2deg))
print("yaw: %f" %(y*rad2deg))
print("------------------------------------")
print("****OUTPUT****")
print("Cal.Deg1: %f" %DEG_INV[0])
print("Cal.Deg2: %f" %DEG_INV[1])
print("Cal.Deg3: %f" %DEG_INV[2])
print("Cal.Deg4: %f" %DEG_INV[3])
print("Cal.Deg5: %f" %DEG_INV[4])
print("Cal.Deg6: %f" %DEG_INV[5])
print("")
print("Servo Deg1: %f" %ServoAng1)
print("Servo Deg2: %f" %ServoAng2)
print("Servo Deg3: %f" %ServoAng3)
print("Servo Deg4: %f" %ServoAng4)
print("Servo Deg5: %f" %ServoAng5)
print("Servo Deg6: %f" %ServoAng6)

'''
B1 = numpy.array([0, -wb, 0])
B2 = numpy.array([(root3/2)*wb, 0.5*wb, 0])
B3 = numpy.array([(-root3/2)*wb, 0.5*wb, 0])

b1 = numpy.array([sb/2, -wb, 0])
b2 = numpy.array([0, ub, 0])
b3 = numpy.array([-sb/2, -wb, 0])

P1 = numpy.array([0, -up, 0])
P2 = numpy.array([sp/2, wp, 0])
P3 = numpy.array([-sp/2, wp, 0])

########################### Draw circular path ############################
r = 50.0					# Input the radius of a circle 400 is max.
zeta = [None]*360
X = [None]*360
Y1 = [None]*180
Y2 = [None]*180
Y = [None]*360

for i in range(1,361):
    zeta[i-1]=i*deg2rad
    X[i-1] = r*math.cos(zeta[i-1])

for i in range(1,(len(X)/2)+1):
    Y1[i-1] = math.sqrt(r**2 - X[i-1]**2)

j = 0
for i in range((len(X)/2)+1,len(X)+1):
    Y2[j] = -math.sqrt(r**2 - X[j]**2)
    j = j+1

Y = numpy.concatenate((Y1,Y2))

####################### Inverse Kinematics ############################
delaySpeed = 0.03
rest = 0.7
for i in range(0,359):
	x = X[i]
	y = Y[i]
	z = -485   # home is -485 but actual length is -460 but the difference 

	a = wb - up
	b = sp/2 - (root3/2)*wb
	c = wp - 0.5*wb

	E1 = 2*L*(y + a)
	F1 = 2*z*L
	G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2

	E2 = -L*(root3*(x+b) + y + c)
	F2 = 2*z*L
	G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b + y*c) - l**2

	E3 = L*(root3*(x-b) - y - c)
	F3 = 2*z*L
	G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b + y*c) - l**2;

	t1 = [None]*2
	t2 = [None]*2
	t3 = [None]*2

	t1[0] = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
	t1[1] = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
	t2[0] = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
	t2[1] = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
	t3[0] = (-F3 + math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)
	t3[1] = (-F3 - math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)

	theta1 = 2*math.atan(t1[1]);
	theta2 = 2*math.atan(t2[1]);
	theta3 = 2*math.atan(t3[1]);

	deg1 = theta1*rad2deg;
	deg2 = theta2*rad2deg;
	deg3 = theta3*rad2deg;

	if isinstance(theta1, complex) or isinstance(theta2, complex) or isinstance(theta3, complex):
		print("Error: Driving angle is complex number")
		break

	servo = maestro.Controller()
	pos0 = servo.getPosition(0)
	pre_pos = map(pos0,4000, 8000, -70, 20)

	pos0 = servo.getPosition(0)

	acc = 20
	servo.setAccel(1,acc)      #set servo 0 acceleration to 4
	servo.setAccel(1,acc)      #set servo 0 acceleration to 4
	servo.setAccel(2,acc)      #set servo 0 acceleration to 4
	
	speed = 200
	servo.setSpeed(0,speed)
	servo.setSpeed(1,speed)
	servo.setSpeed(2,speed)
	
	#pos = 60
	pos1 = -deg1
	pos2 = -deg2
	pos3 = -deg3

	minAng = -70.0
	maxAng = 20.0
	offset1 = 400
	offset2 = 600
	offset3 = 400

	servo_pos = map(pos1, minAng, maxAng, 4000, 8000)
	servo_pos2 = map(pos2, minAng, maxAng, 4000, 8000)
	servo_pos3 = map(pos3, minAng, maxAng, 4000, 8000)
	t1 = time.time()
	servo.setTarget(0,int(servo_pos) - offset1)  #set servo to move to center position  min=3000  mid=6000  max=9000
	servo.setTarget(1,int(servo_pos2) - offset2)
	servo.setTarget(2,int(servo_pos3) - offset3)
	time.sleep(delaySpeed)

pos1 = 0
pos2 = 0
pos3 = 0
servo_pos = map(pos1, minAng, maxAng, 4000, 8000)
servo_pos2 = map(pos2, minAng, maxAng, 4000, 8000)
servo_pos3 = map(pos3, minAng, maxAng, 4000, 8000)
servo.setTarget(0,int(servo_pos) - offset1)  #set servo to move to center position  min=3000  mid=6000  max=9000
servo.setTarget(1,int(servo_pos2) - offset2)
servo.setTarget(2,int(servo_pos3) - offset3)
time.sleep(rest)


servo.close

'''