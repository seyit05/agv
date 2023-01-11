#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
AGV Kontrol
---------------------------
Moving around:
   u    w    e
   a    s    d
   z    x    c

t/g : increase/decrease max speeds by 10%
y/h : increase/decrease only linear speed by 10%
u/j : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'e':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        'q':(1,1),
        'x':(-1,0),
        'c':(-1,1),
        'z':(-1,-1),
           }

speedBindings={
  #      't':(1.1,1.1),
  #      'g':(.9,.9),
        'y':(1.1,1),
        'h':(0.9,1),
        'u':(1,1.1),
        'j':(1,.9),
          }

def getKey(self):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Test():
    speed = 0.3
    turn = 0.2
    key = ''
    linearVelocityData = 0.3
    angularVelocityData = 0.2

    def callback(self, msg):
        self.key = msg.data

    def callback2(self, msg2):
        self.linearVelocityData = msg2.data

    def callback3(self, msg3):
        self.angularVelocityData = msg3.data


    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed,self.turn)

    def rub(self):
        sub1 = rospy.Subscriber('/hiz_mesaji', String, self.callback)
        sub2 = rospy.Subscriber('/angularVelocityData', Float64, self.callback3)
        sub3 = rospy.Subscriber('/linearVelocityData', Float64, self.callback2)
        settings = termios.tcgetattr(sys.stdin)
        
        rospy.init_node('teleop1')
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        x = 0
        th = 0
        status = 0
        count = 0
        acc = 0.1
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        try:
            print(msg)
            print(self.vels())
            while not rospy.is_shutdown():
                #key = getKey()
                if self.key in moveBindings.keys():
                    x = moveBindings[self.key][0]
                    th = moveBindings[self.key][1]
                    count = 0
                elif self.key in speedBindings.keys():
                    self.linearVelocityData = self.linearVelocityData * speedBindings[self.key][0]
                    self.turn = self.turn * speedBindings[self.key][1]
                    count = 0

                    print(self.vels())
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                elif self.key == ' ' or self.key == 's' :
                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                else:
                    count = count + 1
                    if count > 4:
                        x = 0
                        th = 0
                    if (self.key == '\x03'):
                        break

                target_speed = self.linearVelocityData * x
                target_turn = self.angularVelocityData * th

                if target_speed > control_speed:
                    control_speed = min( target_speed, control_speed + 0.00002 )
                elif target_speed < control_speed:
                    control_speed = max( target_speed, control_speed - 0.00002 )
                else:
                    control_speed = target_speed

                if target_turn > control_turn:
                    control_turn = min( target_turn, control_turn + 0.1 )
                elif target_turn < control_turn:
                    control_turn = max( target_turn, control_turn - 0.1 )
                else:
                    control_turn = target_turn

                twist = Twist()
                twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
                pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    aa = Test()
    aa.rub()

