#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>



#define MAX_VEL 			0.5//m/s //launchファイルでなにも指定しなかったときの値
#define MIN_VEL 			-0.5//m/s
#define MAX_ANGULAR_VEL 	2.3//m/s
#define MIN_ANGULAR_VEL 	-2.3//m/s

#define  SERVO_WAIT  		1
#define  SERVO_PREPARE  	2
#define  SERVO_GEREGE_PASS  3
#define  SERVO_SHAGAI_GET  	4
#define  SERVO_SHAGAI_SHOOT 5
#define  SERVO_CLOSE  		6

#define  VALVE_WAIT         1
#define  VALVE_SHAGAI_PUSH  2
#define  VALVE_SHAGAI_PULL  3
#define  VALVE_GEREGE_GET   4
#define  VALVE_GEREGE_PASS  5

#define  ARM_UP_POW  		25
#define  ARM_SHAGAI_GET_POW 20

ros::Publisher twist_pub;
ros::Publisher arm_deg_pub;
ros::Publisher shagai_get_motor_pub;
ros::Publisher servo_task_pub;
ros::Publisher valve_task_pub;

ros::Subscriber joy_sub;
ros::Subscriber servo_sub;
ros::Subscriber valve_sub;


int servo_state = SERVO_WAIT;
int valve_state = VALVE_WAIT;
float now_arm_deg = 0.0;

class Joystick
{
public:
	float linear_x;
	float linear_y;
	float right_spin;
	float left_spin;
	float shagai_shoot;
	float gerege_pass;
	float shagai_get;
	float arm_up_down;
};

Joystick Joystick;

void joy_Callback(const sensor_msgs::Joy& joy){

  Joystick.linear_x = joy.axes[1];//左のスティック上下
  Joystick.linear_y = joy.axes[0];//左のスティック左右

  Joystick.right_spin = joy.buttons[4];//L2ボタン(コントローラの5番)
  Joystick.left_spin  = joy.buttons[5];//R2ボタン(コントローラの6番)

//  Joystick.arm_up = joy.axes[3];//右のスティック上下

  Joystick.shagai_shoot = joy.buttons[2];//右の3番

//  Joystick.air_servo = joy.buttons[3];//右の4番 

  Joystick.gerege_pass = joy.buttons[0];//右の1番 

  Joystick.shagai_get = joy.buttons[1];//右の2番

  Joystick.arm_up_down = joy.axes[5];//左の十字キー上下


}


void servoTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  servo_state = m->data;
}

void valveTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  valve_state = m->data;
}


float map(float x, float in_min, float in_max, float out_min, float out_max)
{//arduinoのmap関数と同じ
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(int argc, char** argv)
{
	int servo_count = 0;
	int valve_count = 0;
	int shoot_count = 0;
	int shagai_get_count = 0;

	int pre_shagai_get_button_state;


	ros::init(argc, argv, "joy");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	float MAX_VEL_param=MAX_VEL;
	float MIN_VEL_param=MIN_VEL;
	float MAX_ANGULAR_VEL_param=MAX_ANGULAR_VEL;
	float MIN_ANGULAR_VEL_param=MIN_ANGULAR_VEL;

	pn.getParam("MAX_VEL",MAX_VEL_param);
	pn.getParam("MIN_VEL",MIN_VEL_param);
	pn.getParam("MAX_ANGULAR_VEL",MAX_ANGULAR_VEL_param);
	pn.getParam("MIN_ANGULAR_VEL",MIN_ANGULAR_VEL_param);

	joy_sub = n.subscribe("joy", 1, joy_Callback);
	servo_sub = n.subscribe("servo_task", 1, servoTaskCallback);
	valve_sub = n.subscribe("valve_task", 1, valveTaskCallback);

	twist_pub = n.advertise<geometry_msgs::Twist>("sub",1);
	arm_deg_pub = n.advertise<std_msgs::Int16>("arm_pow",1);
	shagai_get_motor_pub = n.advertise<std_msgs::Int16>("shagai_get",1);
	servo_task_pub = n.advertise<std_msgs::Int8>("servo_task",1);
	valve_task_pub = n.advertise<std_msgs::Int8>("valve_task",1);



	ros::Rate r(10.0);

	std_msgs::Int16 deg;
	std_msgs::Int8 servo_task;
	std_msgs::Int8 valve_task;

//	std_msgs::Bool vacuum_task;
	std_msgs::Int16 shagai_get_motor_msg;


	while(n.ok()){

		ros::spinOnce();
//		ROS_INFO("o");
		geometry_msgs::Twist twist;
  //  int temp_deg = 0;

		twist.linear.x = map(Joystick.linear_x,-1,1,MIN_VEL_param,MAX_VEL_param);
		twist.linear.y = map(Joystick.linear_y,-1,1,MIN_VEL_param,MAX_VEL_param);

		if(Joystick.right_spin == 1)
		{
			twist.angular.z = MAX_ANGULAR_VEL;
		}    
		else if(Joystick.left_spin == 1)
		{
			twist.angular.z = MIN_ANGULAR_VEL;
		}    



		if(Joystick.shagai_get == 1 && pre_shagai_get_button_state == 0)
		{
			switch(shagai_get_count)
			{
				case 0:
					shagai_get_motor_msg.data = -ARM_SHAGAI_GET_POW;
					shagai_get_motor_pub.publish(shagai_get_motor_msg);
					shagai_get_count++;
					break;

				case 1:
					shagai_get_motor_msg.data = ARM_SHAGAI_GET_POW-5;
					shagai_get_motor_pub.publish(shagai_get_motor_msg);
					shagai_get_count = 0;
					break;
			}
		}    

		pre_shagai_get_button_state = Joystick.shagai_get;

		if(Joystick.arm_up_down == 1)
		{
			deg.data = ARM_UP_POW;
			arm_deg_pub.publish(deg);
		}
		else if(Joystick.arm_up_down == -1)
		{
			deg.data = -ARM_UP_POW;
			arm_deg_pub.publish(deg);
		}
		else if(Joystick.arm_up_down == 0)
		{
			deg.data = 0;
			arm_deg_pub.publish(deg);
		}



		if(Joystick.gerege_pass == 1 && valve_state == VALVE_WAIT)
		{
			valve_task.data = VALVE_GEREGE_PASS;
			valve_task_pub.publish(valve_task);
			valve_state = VALVE_GEREGE_PASS;
		}

		switch(shoot_count)
		{
			case 0:
				if(Joystick.shagai_shoot == 1 && valve_state == VALVE_WAIT && servo_state == SERVO_WAIT)
				{
					valve_task.data = VALVE_SHAGAI_PUSH;
					valve_task_pub.publish(valve_task);		
					valve_state = VALVE_SHAGAI_PUSH;
					shoot_count = 1;
				}
				break;

			case 1:
				if(valve_state == VALVE_WAIT)
				{
					servo_task.data = SERVO_SHAGAI_SHOOT;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_SHOOT;
					shoot_count = 2;					
				}
				break;

			case 2:
				if(servo_state == SERVO_WAIT)
				{
					valve_task.data = VALVE_SHAGAI_PULL;
					valve_task_pub.publish(valve_task);		
					valve_state = VALVE_SHAGAI_PULL;
					shoot_count = 0;					
				}
				break;
		}

		twist_pub.publish(twist);

		r.sleep();

//		ROS_INFO("servo_state = %d",servo_state);
//		ROS_INFO("pre_servo_button_state = %f",pre_servo_button_state);
//		ROS_INFO("Joystick.air_servo = %f",Joystick.air_servo);


	}

}
