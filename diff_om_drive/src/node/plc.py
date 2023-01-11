#!/usr/bin/env python3

# --------------------------------------------------------------------------- #
# import the modbus libraries we need
# --------------------------------------------------------------------------- #
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from twisted.internet.task import LoopingCall


import argparse
import sys , os , time




from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, BatteryState
from actionlib_msgs.msg import GoalStatusArray
from ros_msgs.msg import RobotPose, ModbusPLC
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool , Float64


import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.ERROR)

""" 64/16=4    12x0.5(8bit)=6  20*2=40(10 readable)  =50"""
LENGTH_BIT_DATA = 64 
LENGTH_SINT_DATA = 12
LENGTH_FLOAT_DATA = 20

# ----------------------------------------------------------------------- #

# ROS to PLC
"""
Bit DATA
0: Watchdog 
1: Initilaze Data Modbus started waiting for reply from plc
2: Ready
SInt DATA
0: Move_base_status 0 nothing 1 go 2 nothing 3 goal reached 4 error
Float DATA
0: Left wheel velocity rad/s
1: Right wheel velocity rad/s 
2: robot_pose_x
3: robot_pose_y
4: robot_pose_yaw
"""

# PLC to ROS
"""
Bit DATA
0: Watchdog 
1: Initilaze -- if true publish_init_pose 
2: Ready 
SInt DATA
0: Battery
Float DATA
0: Odom Pose x m 320
1: Odom Pose y m 324 
2: Odom Pose yaw rad 328
3: Odom Velocity x m/s 332
4: Odom Velocity y m/s 336 not used
5: Odom Velocity yaw rad/s 340
6: Robot Pose x 344
7: Robot Pose y 348
8: Robot Pose yaw 352 
9:
10: 
11: 
"""

class Modbus(Node):

    def __init__(
        self,
        ip: str,
        port: int = 5020):

        self.ip = ip 
        self.port = port 
        super().__init__(node_name='modbus' )
       

    def virtual_init(self):
        context = self.init_modbus_registers()
        self.init_ros_topics()
        self.run_updating_server(context)


    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')


    def init_ros_topics(self):
        qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        """ Publisher """
        self.modbus_pub = self.create_publisher(ModbusPLC,'modbusPlcData', 10)
        self.battery_pub = self.create_publisher(BatteryState,'battery', 10)

        """ Subscription """
        self.modbus_dummy_sub = self.create_subscription(String, 'modbus_dummy', self.modbus_dummy_callback, qos._depth)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, qos._depth)
        self.robot_pose_sub = self.create_subscription(RobotPose, 'robotpose', self.robot_pose_callback, qos._depth)
        self.move_base_status_sub = self.create_subscription(GoalStatusArray, 'move_base/status', self.move_base_status_callback, qos._depth)
        self.ros_ready_sub = self.create_subscription(Bool, 'ros_ready', self.ros_ready_callback, qos._depth)

        """ Timer """
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def modbus_dummy_callback(self,subscribe_data:String):
        pass

    """ **************************   Bool  ************************** """
    def ros_ready_callback(self,subscribe_data:Bool):
        self.temp_bool[2] = subscribe_data.data
   

    """ **************************   Sint  ************************** """
    def move_base_status_callback(self,subscribe_data:GoalStatusArray):
        if len(subscribe_data.status_list) != 0 :
            if len(subscribe_data.status_list) == 2 :
                # 1 go 2 nothing 3 goal reached 4 error
                self.temp_sint[0] = subscribe_data.status_list[1].status
            else:
                self.temp_sint[0] = subscribe_data.status_list[0].status

        else:
            self.temp_sint[0] = 0 #nothing

    """ **************************   Float  ************************** """
    def joint_state_callback(self,subscribe_data:JointState):
        self.temp_float[0] = subscribe_data.velocity[0]
        self.temp_float[1] = subscribe_data.velocity[1]

    def robot_pose_callback(self,subscribe_data:RobotPose):
        self.temp_float[2] = subscribe_data.x
        self.temp_float[3] = subscribe_data.y
        self.temp_float[4] = subscribe_data.theta

    """ **************************   Timer Callback ************************** """
    # if 50hz is not needed
    def timer_callback(self):
        battery_msg = BatteryState()
        battery_msg.percentage = float(self.modbus_plc_msg.battery)
        self.battery_pub.publish(battery_msg)


    """ **************************   Modbus Registers  ************************** """
    def init_modbus_registers(self):
        builder = BinaryPayloadBuilder(byteorder=Endian.Big,
                                wordorder=Endian.Big)

        for i in range(int(LENGTH_BIT_DATA/8)) :
            builder.add_bits([False,False,False,False,False,False,False,False,])

        for i in range(LENGTH_SINT_DATA) :
            builder.add_8bit_int(0)

        for i in range(LENGTH_FLOAT_DATA) :
            builder.add_32bit_float(0.0)

        for i in range(int(LENGTH_BIT_DATA/8)) :
            builder.add_bits([False,False,False,False,False,False,False,False,])

        for i in range(LENGTH_SINT_DATA) :
            builder.add_8bit_int(0)

        for i in range(LENGTH_FLOAT_DATA) :
            builder.add_32bit_float(0.0)

        self.payload = builder.build()

        block = ModbusSequentialDataBlock(1, builder.to_registers())
        store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
        context = ModbusServerContext(slaves=store, single=True)

        self.ReadValues = []
        self.temp_bool = [False]*LENGTH_BIT_DATA
        self.temp_sint = [0]*LENGTH_SINT_DATA
        self.temp_float = [0.0]*LENGTH_FLOAT_DATA

        self.temp_bool[1] = True # Modbus started waiting for reply from plc
 
        return context

    def run_updating_server(self,context):
        # ----------------------------------------------------------------------- #
        # initialize the server information
        # ----------------------------------------------------------------------- #
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'pymodbus'
        identity.ProductCode = 'PM'
        identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        identity.ProductName = 'pymodbus Server'
        identity.ModelName = 'pymodbus Server'
        identity.MajorMinorRevision = '2.3.0'

        # ----------------------------------------------------------------------- #
        # run the server you want
        # ----------------------------------------------------------------------- #
        time = 0.02  # 5 seconds delay
        loop = LoopingCall(f=self.updating_writer, a=(context,))
        loop.start(time, now=False)  # initially delay by time

        try:
            StartTcpServer(context, identity=identity, address=(self.ip,int(self.port)))
        except  Exception as e :
            self.error_msg('Error Starting modbus '+ repr(e) )

    def updating_writer(self,a):
        #log.debug("updating the context")
        context = a[0]
        register = 3
        slave_id = 0x00
        address = 0x00
        count = len(self.payload)
        values = context[slave_id].getValues(register, address, count=count)

        decoder = BinaryPayloadDecoder.fromRegisters(values,
                                                        byteorder=Endian.Big,
                                                        wordorder=Endian.Big)

        del self.ReadValues[:]

        for i in range(8):
            temp = decoder.decode_bits()
            for j in temp:
                self.ReadValues.append(j)

        for i in range(LENGTH_SINT_DATA):
            self.ReadValues.append(decoder.decode_8bit_int())

        for i in range(LENGTH_FLOAT_DATA):
            self.ReadValues.append(decoder.decode_32bit_float())

        for i in range(8):
            temp = decoder.decode_bits()
            for j in temp:
                self.ReadValues.append(j)

        for i in range(LENGTH_SINT_DATA):
            self.ReadValues.append(decoder.decode_8bit_int())

        for i in range(LENGTH_FLOAT_DATA):
            self.ReadValues.append(decoder.decode_32bit_float())  


        """ Update PLC read registers from ROS """
        for idx in range(LENGTH_BIT_DATA):
            self.ReadValues[idx] = self.temp_bool[idx]

        temp_start_index = LENGTH_BIT_DATA
        temp_end_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA
        for idx in range(temp_start_index,temp_end_index):
            self.ReadValues[idx] = self.temp_sint[idx-temp_start_index]

    
        temp_start_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA
        temp_end_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA+LENGTH_FLOAT_DATA
        for idx in range(temp_start_index,temp_end_index):
            self.ReadValues[idx] = self.temp_float[idx-temp_start_index]
    
        """ Watchdog with PLC """
        watchdog_data_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA +LENGTH_FLOAT_DATA

        if self.ReadValues[watchdog_data_index]:
            self.ReadValues[0]=True
        else:
            self.ReadValues[0]=False

        """ Comm Init bit with PLC -- Publish InitialPose"""
        if self.ReadValues[watchdog_data_index+1]:
            self.ReadValues[1]=False
            self.temp_bool[1]=False
            

        """ Create of Write Registers """
        senddata = BinaryPayloadBuilder(byteorder=Endian.Big,
                                        wordorder=Endian.Big)

        for i in range(int(LENGTH_BIT_DATA/8)):
            temp = []
            for j in range(8):
                temp.append(self.ReadValues[j+i*8])     
            senddata.add_bits(temp)

        temp_start_index = LENGTH_BIT_DATA
        temp_end_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA
        for i in range(temp_start_index,temp_end_index) :
            senddata.add_8bit_int(self.ReadValues[i])

        temp_start_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA
        temp_end_index = LENGTH_BIT_DATA+LENGTH_SINT_DATA+LENGTH_FLOAT_DATA
        for i in range(temp_start_index,temp_end_index) :
            senddata.add_32bit_float(self.ReadValues[i])

        senddatapayload = senddata.to_registers()

        context[slave_id].setValues(register, address, senddatapayload)

        start_second_half_index = LENGTH_BIT_DATA + LENGTH_SINT_DATA + LENGTH_FLOAT_DATA 
        self.modbus_plc_msg = self.create_modbus_plc_msg_func(self.ReadValues[start_second_half_index:len(self.ReadValues)])
        self.modbus_pub.publish(self.modbus_plc_msg)
        rclpy.spin_once(self)

    def create_modbus_plc_msg_func(self,data:list)->ModbusPLC:
        msg = ModbusPLC()
        """ Bool """
        msg.watchdog = data[0]
        msg.comm_init = data[1]
        msg.ready = data[2]
        msg.others_bool = data[3:LENGTH_BIT_DATA]
        """ Int """
        idx_int = LENGTH_BIT_DATA 
        msg.battery = data[idx_int]
        msg.others_int = data[idx_int+1:(idx_int+LENGTH_SINT_DATA)]
        """ Float """
        idx_float = LENGTH_BIT_DATA + LENGTH_SINT_DATA
        msg.odom_pose = [ data[idx_float] , data[idx_float+1] , data[idx_float+2] ]
        msg.odom_velocity = [ data[idx_float+3] , data[idx_float+4] , data[idx_float+5] ]
        msg.robot_pose = [ data[idx_float+6] , data[idx_float+7] , data[idx_float+8] ]
        msg.others_float = data[idx_float+9:len(data)]

        return msg


def main(args=None):  
    os.system('fuser -k 5020/tcp ')
    
    rclpy.init(args=args)
   
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-i', '--ip', help='modbus server ip adress',
        default='192.168.0.2', required=False)
    parser.add_argument(
        '-p', '--port', help='Define comunication port',
        default='502', required=False)
   
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
 
    modbus = Modbus(ip=args.ip , port=args.port) 
    modbus.info_msg(
        'Starting modbus, connection   ' + args.ip + ' : ' + args.port )
    modbus.virtual_init()

if __name__ == '__main__':
    main()
