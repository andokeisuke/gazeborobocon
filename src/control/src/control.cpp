#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>



#define MAX_VEL 0.5//m/s         //launchファイルでなにも指定しなかったときの値

#define MIN_VEL -0.5//m/s
#define MAX_ANGULAR_VEL 2.3//m/s
#define MIN_ANGULAR_VEL -2.3//m/s

#define  SERVO_WAIT  1
#define  SERVO_PREPARE  2
#define  SERVO_GEREGE_PASS  3
#define  SERVO_SHAGAI_GET  4
#define  SERVO_SHAGAI_SHOOT  5
#define  SERVO_CLOSE  6

#define  VALVE_WAIT  1
#define  VALVE_SHAGAI_PUSH  2
#define  VALVE_SHAGAI_PULL  3

ros::Publisher twist_pub;
ros::Publisher arm_deg_pub;
ros::Publisher gerege_stepping_pub;
ros::Publisher servo_task_pub;
ros::Publisher valve_task_pub;

ros::Subscriber joy_sub;
ros::Subscriber servo_sub;
ros::Subscriber valve_sub;


int servo_state = SERVO_WAIT;
int valve_state = VALVE_WAIT;


class Joystick
{
public:
	float linear_x;
	float linear_y;
	float right_spin;
	float left_spin;
	float arm_up;
	float arm_down;
	float gerege_pass;
	float air_servo;
	float shagai_sylinder;
	float vacuum_motor;
};

Joystick Joystick;

void joy_Callback(const sensor_msgs::Joy& joy){

  Joystick.linear_x = joy.axes[1];//左のスティック上下
  Joystick.linear_y = joy.axes[0];//左のスティック左右

  Joystick.right_spin = joy.buttons[4];//L2ボタン(コントローラの5番)
  Joystick.left_spin  = joy.buttons[5];//R2ボタン(コントローラの6番)

  Joystick.arm_up = joy.axes[3];//右のスティック上下

  Joystick.gerege_pass = joy.buttons[2];//右の3番

  Joystick.air_servo = joy.buttons[3];//右の4番 

  Joystick.shagai_sylinder = joy.buttons[0];//右の1番 

  Joystick.vacuum_motor = joy.buttons[1];//

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

	float pre_servo_button_state = 0;
	float pre_vacuum_button_state = 0;

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
	arm_deg_pub = n.advertise<std_msgs::Int16>("tar_arm_deg",1);
	gerege_stepping_pub = n.advertise<std_msgs::Bool>("gerege_pass_state",1);
	servo_task_pub = n.advertise<std_msgs::Int8>("servo_task",1);
	valve_task_pub = n.advertise<std_msgs::Int8>("valve_task",1);



	ros::Rate r(10.0);

	std_msgs::Int16 deg;
	std_msgs::Int8 servo_task;
	std_msgs::Int8 valve_task;

	std_msgs::Bool vacuum_task;
	std_msgs::Bool gerege_pass_msg;


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


		
		if(Joystick.arm_up == 0 &&Joystick.arm_down == 0)
		{
			deg.data = 0;
		}    
		else if(Joystick.arm_up == 1)
		{
			deg.data = -90;
		}    
		else if(Joystick.arm_down == 1)
		{
			deg.data = 90;
		}    



		if(Joystick.gerege_pass == 1)
		{
			gerege_pass_msg.data = true;
			gerege_stepping_pub.publish(gerege_pass_msg);
		}


		if(pre_servo_button_state == 0 && Joystick.air_servo == 1 && servo_state == SERVO_WAIT)
		{
			switch (servo_count)
			{
				case 0:
				
					servo_task.data = SERVO_PREPARE;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_PREPARE;
					servo_count = 1;
					break;
				
/*
					servo_task.data = SERVO_SHAGAI_SHOOT;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_SHOOT;
					servo_count = 0;
					break;
*/

				case 1:	
/*
					servo_task.data = SERVO_GEREGE_PASS;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_GEREGE_PASS;
					servo_count = 2;
					break;
*/
					servo_task.data = SERVO_SHAGAI_SHOOT;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_SHOOT;
					servo_count = 1;
					break;

				case 2:	
					servo_task.data = SERVO_SHAGAI_GET;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_GET;
					servo_count = 3;
					break;

				case 3:	
					servo_task.data = SERVO_SHAGAI_SHOOT;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_SHOOT;
					servo_count = 2;
					break;


			}

		}

		if(Joystick.shagai_sylinder == 1 && valve_state == VALVE_WAIT)
		{
			switch (valve_count)
			{
				case 0:
				
					valve_task.data = VALVE_SHAGAI_PUSH;
					valve_task_pub.publish(valve_task);		
					valve_state = VALVE_SHAGAI_PUSH;
					valve_count = 1;
					break;
				
				case 1:	

					valve_task.data = VALVE_SHAGAI_PULL;
					valve_task_pub.publish(valve_task);		
					valve_state = VALVE_SHAGAI_PULL;
					valve_count = 0;
					break;
			}

		}

		arm_deg_pub.publish(deg);

		twist_pub.publish(twist);

		pre_servo_button_state = Joystick.air_servo;
		pre_vacuum_button_state= Joystick.vacuum_motor;
		r.sleep();

//		ROS_INFO("servo_state = %d",servo_state);
//		ROS_INFO("pre_servo_button_state = %f",pre_servo_button_state);
//		ROS_INFO("Joystick.air_servo = %f",Joystick.air_servo);


	}

}
