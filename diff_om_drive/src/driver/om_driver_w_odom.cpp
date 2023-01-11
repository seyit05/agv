#include "math.h"
#include "modbus.h"
#include "modbus-rtu.h"
#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

/**
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


												DEFINITIONS, DEFINITIONS EVERYWHHERE


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**/

//DRIVETRAIN PARAMS
#define R			0.1	//RADIUS (M)
#define L			0.505		//DISTANCE BT WHEELS (M)
#define COEFF		286.47889		//30/pi * disliorani ----30:1 disli oranı

//MODBUS RTU PARAMS
#define PORT_RIGHT		"/dev/sag_motor"	
#define PORT_LEFT			"/dev/sol_motor"		
#define BAUD_RATE			57600	
#define PARITY				'E'	
#define DATA_BITS			8
#define STOP_BITS			1
#define SLAVE_ID			1		

//DRIVER WRITE ADD AND DATA
#define REG_DRIVER_INPUT				125	// FOLLOWING DATA DEFS ARE LIST OF ALLOWED
#define DATA_DI_FORWARD					42
#define DATA_DI_BACKWARD				50
#define DATA_DI_FORWARD_FREE_B		170
#define DATA_DI_BACKWARD_FREE_B		178
#define DATA_DI_STOP_DEC_B_ON			34
#define DATA_DI_STOP_EC_B_OFF			130
#define DATA_DI_STOP_EM_B_ON			2
#define DATA_DI_STOP_EM_B_OFF			162

#define REG_CLEAR_ALARM_RECORDS		389	// WRITE FIRST 1, THEN 0
#define REG_CLEAR_WARNING_RECORDS 	391	// WRITE FIRST 1, THEN 0
#define REG_CLEAR_COM_ERR_RECORDS	393	// WRITE FIRST 1, THEN 0
#define REG_RESET_ALARM					385	// WRITE FIRST 1, THEN 0
#define REG_ROTATIONAL_VELOCITY		1157	//	0:80-4000 RPM
#define REG_TORQUE_LIMITING			1797	//	0-200 (%)
#define REG_DECELERATION_TIME			1669	//	2-150 (1 = .1SEC)
#define REG_ACCELARATION_TIME			1541	//	2-150 (1 = .1SEC)

//DRIVER READ ADD AND DATA	(SIGNED: TRUE)
#define REG_SPEED_FEEDBACK				206	//	UPPER HOLDING REGISTER, USED WITH LOWER TO DETERMINE W IN DIRECTION

#define LIM_RPM_MIN						80
#define LIM_RPM_MAX						4000

#define ODOM_PUB_RATE					100	//	IN LOOP CYCLES

/**
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


														VARIABLES HERE ARE


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**/
//Modbus
bool comNonInitiated = true;

modbus_t *motorLeft;
modbus_t *motorRight;

//States
int 	stateMotorRight	= 0;
int 	stateMotorLeft		= 0;
int 	command				= 0;
int 	commandFlag			= 0;

bool 	frenAc		= true;
bool	frenFlag		= false;

float	WRk1		= 0;
float	WLk1		= 0;
float	WR			= 0;
float	WL			= 0;

float laserCoF = 1.0;
float laserCoB = 1.0;
float camCoeff = 1.0;

bool estop		= false;
bool estopFlag	= false;

//Odom Publication
int cycleCount= 0;		//	COUNTS LOOP ITERATON

float x 		= 0.0;
float y 		= 0.0;
float th 	= 0.0;

float vx 	= 0.0;
float vy 	= 0.0;
float vth 	= 0.0;

/**
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


														HERE LIES THE METHODS USED


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**/
void initiateModbus(modbus_t **ctx, char *comPort){
	*ctx = modbus_new_rtu(comPort, BAUD_RATE, PARITY, DATA_BITS, STOP_BITS);
	if (*ctx == NULL) {
    	ROS_INFO("Unable to create the libmodbus context\n");
    	return;
	}
	modbus_set_slave(*ctx, SLAVE_ID);

	if (modbus_connect(*ctx) == -1) {
    	ROS_INFO( "Connection failed %s\n", comPort);
    	modbus_free(*ctx);
    	return;
	}
}

void resetRoutine(modbus_t *ctx, int addr){
	if (ctx == NULL) {
    	ROS_INFO("Context does not exists\n");
    	return;
	}
	
	modbus_write_register(ctx, addr, 1);
	ros::Duration(.1).sleep();
	modbus_write_register(ctx, addr, 0);
	ros::Duration(.1).sleep();
}

void clearAlarm(modbus_t *ctx){
	resetRoutine(ctx, REG_CLEAR_ALARM_RECORDS);
}

void clearWarning(modbus_t *ctx){
	resetRoutine(ctx, REG_CLEAR_WARNING_RECORDS);
}

void resetCommErr(modbus_t *ctx){
	resetRoutine(ctx, REG_CLEAR_COM_ERR_RECORDS);
}

void resetAlarm(modbus_t *ctx){
	resetRoutine(ctx, REG_RESET_ALARM);
}

void motorSetRpm(modbus_t *ctx, float rpm){
	if (ctx == NULL) {
    	ROS_INFO("Context does not exists\n");
    	return;
	}
	
	if(rpm == 0){
		modbus_write_register(ctx, REG_ROTATIONAL_VELOCITY, 0);
		return;
	}
	
	if(abs(rpm) < LIM_RPM_MIN){
		rpm = 0;
	} else if(abs(rpm) > LIM_RPM_MAX){
		rpm = LIM_RPM_MAX;
	}else {
		rpm = abs(rpm) + 0.5; //Forcing the Rounding of Double Up Before Casting
	}
	modbus_write_register(ctx, REG_ROTATIONAL_VELOCITY, ((int)rpm));
	return;	
}

void readFeedback(modbus_t *ctx){
	if (ctx == NULL) {
    	ROS_INFO("Context does not exists\n");
    	return;
	}
	
	uint16_t tab_reg[64];
	int rc = modbus_read_registers(ctx, REG_SPEED_FEEDBACK, 2, tab_reg);
	if (rc == -1) {
    	fprintf(stderr, "%s\n", modbus_strerror(errno));
    	return;
	}

	for (int i=0; i < rc; i++) {
    	printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
	}
	return;
}

void velocityFeedback(modbus_t *left, modbus_t *right){
	
	if (left == NULL || right == NULL) {
    	ROS_INFO("Context does not exists\n");
    	return;
	}
	
	uint16_t leftReg[2];
	int lrc = modbus_read_registers(left, REG_SPEED_FEEDBACK, 2, leftReg);
	if (lrc == -1) {
    	fprintf(stderr, "%s\n", modbus_strerror(errno));
    	return;
	}
	
	uint16_t rightReg[2];
	int rrc = modbus_read_registers(right, REG_SPEED_FEEDBACK, 2, rightReg);
	if (lrc == -1) {
    	fprintf(stderr, "%s\n", modbus_strerror(errno));
    	return;
	}

	float WLFeed =   (leftReg[1]  - leftReg[0]); 
	float WRFeed = -(rightReg[1] - rightReg[0]);
	
	WLFeed = (WLFeed / COEFF)*R;
	WRFeed = (WRFeed / COEFF)*R;
	
	vx  = (WLFeed + WRFeed) / 2.0;
	vth = (WRFeed - WLFeed) / L;
	
	return;
}

/**
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


												HERE LIES THE CALLBACK FUNCTIONS


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**/
// Topic messages callback
void velocityCb(const geometry_msgs::Twist::ConstPtr& msg){
	WR = (msg->linear.x + (msg->angular.z*L/2.0))/R;
	WL = (msg->linear.x - (msg->angular.z*L/2.0))/R;
	WR = WR * COEFF * camCoeff * laserCoF;
	WL = WL * COEFF * camCoeff * laserCoF;
}

// Topic messages callback
void diFrenCb(const std_msgs::Bool::ConstPtr& msg){
    frenAc = msg->data;
}

// Topic messages callback
void diK1Cb(const std_msgs::Bool::ConstPtr& msg){
    estop = msg->data;
}

// Topic messages callback
void omCmdCb(const std_msgs::Byte::ConstPtr& msg){   
	command = msg->data;
	commandFlag = 1;
}

// Topic messages callback
void laserCoeffFCb(const std_msgs::Float32::ConstPtr& msg){
	laserCoF = msg->data;
}

// Topic messages callback
void camSafetyCb(const std_msgs::Float32::ConstPtr& msg){
	camCoeff = msg->data;
}

/**
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


																MAIN FOLLOWS


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**/
int main(int argc, char **argv){

	ros::init(argc, argv, "om_driver", ros::init_options::NoSigintHandler);
	
	ros::NodeHandle node;
   
	ros::Subscriber sub1 = node.subscribe("cmd_vel", 		1, velocityCb);
	ros::Subscriber sub2 = node.subscribe("di_fren", 		1, diFrenCb);
	ros::Subscriber sub3 = node.subscribe("/PLC_em_stop", 	1, diK1Cb);
	ros::Subscriber sub4 = node.subscribe("om_commands", 	1, omCmdCb);
	ros::Subscriber sub5 = node.subscribe("laser_coeff_F", 		1, laserCoeffFCb);
	ros::Subscriber sub6 = node.subscribe("cam_safety_coeff", 	1, camSafetyCb);
	
	ros::Publisher odomPub = node.advertise<nav_msgs::Odometry>("odom", 10);
  	tf::TransformBroadcaster odomBroadcaster;
  	
  	ros::Time now, prev;
	now 		= ros::Time::now();
	prev		= ros::Time::now();
	
	std::string odomFrameId;
	node.param<std::string>("odom_frame_id", odomFrameId, "odommo");
			
	std::string baseFrameId;
	node.param<std::string>("base_frame_id", odomFrameId, "odom");		
	
	//Establish Connection, Clear Alarms and Registers
	while(comNonInitiated){
		initiateModbus(&motorLeft , PORT_LEFT );
		initiateModbus(&motorRight, PORT_RIGHT);
		
		if(motorLeft != NULL || motorRight != NULL){
			comNonInitiated = false;
		
			//Warnings and Alarms Cleared for the Initialization
			clearAlarm(motorLeft);
			clearAlarm(motorRight);
			clearWarning(motorLeft);
			clearWarning(motorRight);
			resetAlarm(motorLeft);
			resetAlarm(motorRight);
			resetCommErr(motorLeft);
			resetCommErr(motorRight);
		
			//Setting Motor Parameters for the Initialization
			motorSetRpm(motorLeft,  0);
			motorSetRpm(motorRight, 0);
		
			modbus_write_register(motorLeft,  REG_DECELERATION_TIME, 2);
			modbus_write_register(motorRight, REG_DECELERATION_TIME, 2);
			modbus_write_register(motorLeft,  REG_TORQUE_LIMITING, 	200);
			modbus_write_register(motorRight, REG_TORQUE_LIMITING, 	200);
			modbus_write_register(motorLeft,  REG_ACCELARATION_TIME, 2);
			modbus_write_register(motorRight, REG_ACCELARATION_TIME, 2);
			
			break;
		}
		
		ros::Duration(1).sleep();
	}
	
	while (ros::ok()){
		
		
		if(!estop && !estopFlag){
			motorSetRpm(motorLeft,  0);
			motorSetRpm(motorRight, 0);		
			estopFlag = true;
		}else if (estop && estopFlag){
			resetAlarm(motorLeft);
			resetAlarm(motorRight);
			clearAlarm(motorLeft);
			clearAlarm(motorRight);
			resetCommErr(motorLeft);
			resetCommErr(motorRight);	
			ROS_INFO("estop");
			estopFlag = false;
		}
		
		if (estop && WR > 0 && stateMotorRight != 2){
			stateMotorRight = 2; 
			modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
		}else if (estop && WR < 0 && stateMotorRight != 1){
			stateMotorRight = 1;
			modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_FORWARD);
		}
		
		if (estop && WL > 0 && stateMotorLeft != 1){
			stateMotorLeft = 1; 
			modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_FORWARD);
		}else if (estop && WL < 0 and stateMotorLeft != 2){
			stateMotorLeft = 2;
			modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
		}
		
		if(frenAc){
			if(frenFlag == 1){
				frenFlag = 0;
		
				if(stateMotorRight != 2){
					stateMotorRight = 2;
					modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
				}else if(stateMotorRight != 1){
					stateMotorRight = 1;
					modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_FORWARD);
				}
							
				if(stateMotorLeft != 1){
					stateMotorLeft = 1; 
					modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_FORWARD);
				}else if(stateMotorLeft != 2){
					stateMotorLeft = 2;
					modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
				}
			}
		}else{
			if(frenFlag == 0){
				frenFlag = 1;
				
				if(stateMotorRight != 2){
					stateMotorRight = 2;
					modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
				}else if(stateMotorRight != 1){
					stateMotorRight = 1;
					modbus_write_register(motorRight, REG_DRIVER_INPUT, DATA_DI_FORWARD);
				}

				if(stateMotorLeft != 1){
					stateMotorLeft = 1; 
					modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_FORWARD);
				}else if(stateMotorLeft != 2){
					stateMotorLeft = 2;
					modbus_write_register(motorLeft, REG_DRIVER_INPUT, DATA_DI_BACKWARD);
				}
		
				motorSetRpm(motorLeft,  0);
				motorSetRpm(motorRight, 0);
			}
		}
		
		if(estop){
			if(WL != WLk1){
				motorSetRpm(motorLeft,  WL);
				WLk1 = WL;
			}
			
			if(WR != WRk1){
				motorSetRpm(motorRight,  WR);
				WRk1 = WR;
			}
		}
		
		if(cycleCount == ODOM_PUB_RATE){
			/** readFeedback does not require any more. velocityFeedback succeeds it.**/
			//readFeedback(motorLeft);
			//readFeedback(motorRight);
			cycleCount = 0;
			
			now = ros::Time::now();	
			double dt = (now - prev).toSec();

			nav_msgs::Odometry odom;
			odom.header.stamp = now;
			odom.header.frame_id = odomFrameId;
			odom.child_frame_id = baseFrameId;
			
			/** Velocity Feedback Handles Reading Velocity into Odom.Twist Completly**/
			velocityFeedback(motorLeft, motorRight);
			
			odom.twist.twist.linear.x  = vx;
			odom.twist.twist.linear.y  = 0.0;
			odom.twist.twist.linear.z  = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			odom.twist.twist.angular.z = vth;
			
			if(vth != 0){
				double delta_th = vth * dt;
				th += delta_th;
			}
			
			if(vx != 0){
				double delta_x = (vx * cos(th)/* - vy * sin(th)*/) * dt;
				double delta_y = (vx * sin(th)/* + vy * cos(th)*/) * dt;
			
				x += delta_x;
 				y += delta_y;
 			}
			
			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th);
			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped trans;
			trans.header.stamp = now;
			
			trans.header.frame_id = odomFrameId;
			trans.child_frame_id  = baseFrameId;
			trans.transform.translation.x = x;
			trans.transform.translation.y = y;
			trans.transform.translation.z = 0.0;
			trans.transform.rotation = quat;
			
			//send the transform
			//broadcaster.sendTransform(trans);
  
			//set the position
			odom.pose.pose.position.x 	= x;
			odom.pose.pose.position.y 	= y;
			odom.pose.pose.position.z 	= 0.0;
			odom.pose.pose.orientation = quat;
			odom.pose.covariance = {0.005,0.0,0.0,0.0,0.0,0.0,
            							 0.0,0.05,0.0,0.0,0.0,0.0,
           		 						 0.0,0.0,0.05,0.0,0.0,0.0,
            							 0.0,0.0,0.0,0.01,0.0,0.0,
           								 0.0,0.0,0.0,0.0,0.01,0.0,
            							 0.0,0.0,0.0,0.0,0.0,0.01};

 
			//publish the message
			odomPub.publish(odom);

			prev = now;
		}
		
		cycleCount ++;
		
		ros::spinOnce();
	}	

	return 0;
}
