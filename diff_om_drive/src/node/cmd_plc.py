#! /usr/bin/python3

import rospy
import snap7
import struct
import math
import time
from std_msgs.msg import *
from tf import TransformBroadcaster
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import usb.core
import usb.backend.libusb1
import array
import ctypes


uint16_t = ctypes.c_ushort
ip = '192.168.0.1'
db = 5
L = 0.505
R = 0.1
COEFF = 286.47889
plc = snap7.client.Client()
plc.connect(ip, 0, 1)
estop = bool
estop = False
estopFlag = bool
estopFlag = False
stateMotorRight = int
stateMotorRight = 0
stateMotorLeft = int
stateMotorLeft = 0
command = int
command = 0
commandFlag = int
commandFlag = 0
frenAc = bool
frenAc	= True
frenFlag = bool
frenFlag = False
float = WRk1 = 0
float = WLk1 = 0
float = WR	= 0
float = WL = 0
float = laserCoF = 1.0
float = laserCoB = 1.0
float = camCoeff = 1.0
float = x 		= 0.0
float = y 		= 0.0
float = th 	= 0.0
PORT_RIGHT = "/dev/sag_motor"
PORT_LEFT = "/dev/sol_motor"
LIM_RPM_MIN	= 80					
LIM_RPM_MAX = 4000
ODOM_PUB_RATE = 100
i = int 
i = 0
bool = comNonInitiated = True
tab_reg = (range(64)) 
tab_reg = uint16_t


float = vx 	= 0.0
float = vy 	= 0.0
float = vth = 0.0
int = cycleCount = 0
leftRpm = float = 0
rightRpm = float = 0
leftSet = float = 0
rightSet = float = 0

def motorLeft(leftset):
    leftset1 = struct.pack('>f', leftSet)
    plc.db_write(db, 0, leftset1)

def motorRight(rightSet):
    rightSet1 = struct.pack('>f', rightSet)
    plc.db_write(db, 6, rightSet1)

def motorSetRpm(leftRpm,rightRpm):
    if plc.get_connected() == False:
        motorLeft(0)
        motorRight(0)

    elif(rightRpm == 0):
        motorRight(0)

    elif(leftRpm == 0):
        motorLeft(0)
        

    elif abs(leftRpm) < LIM_RPM_MIN:
        motorLeft(0)
    
    elif abs(rightRpm) < LIM_RPM_MIN:
        motorRight(0)

    elif abs(leftRpm) > LIM_RPM_MAX:
        motorLeft(LIM_RPM_MAX)

    elif abs(rightRpm) > LIM_RPM_MAX:
        motorRight(LIM_RPM_MAX)

    else :
        motorLeft(leftRpm)
        motorRight(rightRpm)

    return


 #   def readFeadback():
 #       i = 0
 #       tab_reg[i] = [plc.db_read(db, 10)]

        


#float WLFeed =   (leftReg[1]  - leftReg[0]) 
#float WRFeed = -(rightReg[1] - rightReg[0])
	
#WLFeed = (WLFeed / COEFF)*R
#WRFeed = (WRFeed / COEFF)*R
	
#vx  = (WLFeed + WRFeed) / 2.0
#vth = (WRFeed - WLFeed) / L


def velocityCb(msg):
    global WR
    global WL
    #msg = Twist()
    WR = msg.linear.x + ((msg.linear.z*L/2.0))/R
    WL = msg.linear.x + ((msg.linear.z*L/2.0))/R
    WR = WL * COEFF * camCoeff * laserCoF
    WL = WR * COEFF * camCoeff * laserCoF
    
  #  WR = float
   # enable = struct.pack('>f',WR)
   # plc.db_write(db, 0, enable)

def diFrenCb(msg):
    global frenAc
  #  msg = Bool()
    frenAc = msg.data 
  #  frenAc = bool(msg.data)
  #  enable = struct.pack('>?', frenAc)
   # plc.db_write(db, 4, enable)

def diK1Cb(msg):
    global estop
    global estopFlag
 #   msg = Bool()
    estop = msg.data
   # estop = bool(estop)
    enable = struct.pack('>?', estop)
    plc.db_write(db, 4, enable)

def omCmdCb(msg):
    global command
    global commandFlag
   # msg = Bool()
    command = msg.data
   # command = bool(msg.data)
    commandFlag = 1
    #enable = struct.pack('>?', command)
    #plc.db_write(db, 4.2, enable)

def laserCoeffFCb(msg):
    global laserCoF
 #   msg = Float32()
    laserCoF = msg.data
   # laserCoF = float(msg.data)
    #enable = struct.pack('>f', laserCoF)
    #plc.db_write(db, 6, enable)

def camSafetyCb(msg):
    global camCoeff
   # msg = Float32()
    camCoeff = msg.data
 #   camCoeff = float(msg.data)
   # enable = struct.pack('>f', camCoeff)
   # plc.db_write(db, 10, enable)



def station(msg):
    global station_number 
    
    station_number = msg.data
   # station_number = msg.data
    
    number = struct.pack('>f', station_number)
    plc.db_write(db, 14, number)
 

def main():

    rospy.init_node("om_driver_python", anonymous=True)
    rospy.Subscriber('/chatter', Float64,station, queue_size=100)
    rospy.Subscriber('di_fren', Bool,diFrenCb, queue_size=100)
    rospy.Subscriber('/PLC_em_stop', Bool,diK1Cb, queue_size=100)
    rospy.Subscriber('om_commands', Bool,omCmdCb, queue_size=100)
    rospy.Subscriber('laser_coeff_F', Float32,laserCoeffFCb, queue_size=100)
    rospy.Subscriber('cam_safety_coeff', Float32,camSafetyCb, queue_size=100)
    rospy.Subscriber('cmd_vel', Twist,velocityCb, queue_size=100)
    station_number = Float64()
 #   odomPub = rospy.Publisher('odom',Odometry, queue_size=5)
 #   odom_frame_id = String()
 #   base_frame_id = String()
 #   odom_frame_id = rospy.get_param("~odom_frame_id", "odommo")
 #   base_frame_id = rospy.get_param("~base_frame_id", "odom")
    
  #  rospy.spin()

    prev = rospy.Time.now().to_sec()
    now = rospy.Time.now().to_sec()

    while(comNonInitiated):

        if plc.get_connected() == True:
            print('Plc is Connected!')
        elif plc.get_connected() == False:
            print('Plc is NOT Connected!')
            comNonInitiated = False
            break

        if (estop != True & estopFlag != True) :
            motorSetRpm(0,0)
            estopFlag == False
            #clearAlarm(motorLeft)
			#clearAlarm(motorRight)
			#clearWarning(motorLeft)
			#clearWarning(motorRight)
			#resetAlarm(motorLeft)
			#resetAlarm(motorRight)
			#resetCommErr(motorLeft)
			#resetCommErr(motorRight)
        
        #elif (estop != True & estopFlag != True) :
        #    motorSetRpm(0,0)
        #    print("estop")
        #    estopFlag = False

        if (frenAc == True) :
            if (frenFlag == True):
                frenFlag == False
        elif (frenAc == False) :
            if(frenFlag == False):
                frenFlag == True
            motorSetRpm(0,0)
            
        if(estop == True):
            if (WL != WLk1):
                motorSetRpm(WL,0)
                WL = WLk1
                 
            elif (WR != WRk1):
                motorSetRpm(0,WR)
                WRk1 =WR

            
            

        


if __name__ == "__main__":
    main()