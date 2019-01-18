#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#define D 0.652//0.652//mトレッド幅　2つの車輪の接地面の中心間距離
#define R 0.125//0.125//m
#define MOTOR_ONE_SPIN_PULSE 23000
#define GEAR_RATIO 3.33
#define WHEEL_ONE_SPIN_PULSE (MOTOR_ONE_SPIN_PULSE * GEAR_RATIO)

#define right_KP 2.8//15
#define right_KI 0 //5
#define right_KD 1//10.5

#define left_KP 4.0//4.0
#define left_KI 0 //5  15.93
#define left_KD 0.6375//0.6375


#define DELTA_T 0.1

struct info
{
  double target_vel;
  double now_vel;
  long pre_pulse;
  long delta_pulse;
  long pulse_data;
};


struct info right={0};
struct info left={0};
double target_vel = 0.0;
double target_omega = 0.0;

double now_vel = 0.0;
double now_omega = 0.0;

std_msgs::Int16 left_pw;
std_msgs::Int16 right_pw;

double dt = 0.0;


void r_chatterCallback(const std_msgs::Int64::ConstPtr& r_pulse)
{
  //ROS_INFO("receive r_pulse: [%d]",r_pulse->data);
  right.pulse_data = r_pulse->data;
}

void l_chatterCallback(const std_msgs::Int64::ConstPtr& l_pulse)
{
  //ROS_INFO("receive l_pulse: [%d]",l_pulse->data);
  left.pulse_data = l_pulse->data;
}

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("receive vx: [%f]",vx);
  target_vel = msg.linear.x;
  target_omega = msg.angular.z;
}

int r_pid_control(double r_nv, double r_tv)
{
  static double p = 0.0, i = 0.0, d = 0.0;
  static double integral = 0.0;
  static double now_diff = 0.0;
  static double pre_diff = 0.0;
  static double power;
  pre_diff = now_diff;
  now_diff = r_tv - r_nv;
  integral += now_diff;

/*
  if(r_nv -r_tv <0.01 && r_nv -r_tv >-0.01 )
    integral = 0;
*/

  p = right_KP * now_diff;
  i = right_KI * integral * DELTA_T; 
  d = right_KD * (now_diff - pre_diff) /DELTA_T;
  

  power =power + p + i + d;
  if(r_tv<0.001&&r_tv>-0.001&&r_nv<0.001&&r_nv>-0.001)
    power = 0;

  return power;
}

int l_pid_control(double l_nv, double l_tv)
{
  static double p = 0.0, i = 0.0, d = 0.0;
  static double integral = 0.0;
  static double now_diff = 0.0;
  static double pre_diff = 0.0;
  static double power;
  pre_diff = now_diff;
  now_diff = l_tv - l_nv;
  integral += now_diff;

  p = left_KP * now_diff;
  i = left_KI * integral * DELTA_T;
  d = left_KD * (now_diff - pre_diff) / DELTA_T;

    ROS_INFO("p = %f",p);
    ROS_INFO("i = %f",i);
    ROS_INFO("d = %f",d);
  

  power =power + p + i + d;
  if(l_tv<0.001&&l_tv>-0.001&&l_nv<0.001&&l_nv>-0.001)
  {
    power = 0;
  ROS_INFO("power = %f",power);
  }
  
  return power;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_to_power");
  
  ros::NodeHandle n;

  ros::Subscriber sub_r_pulse = n.subscribe("r_pulse", 100, r_chatterCallback);
  ros::Subscriber sub_l_pulse = n.subscribe("l_pulse", 100, l_chatterCallback);
  
  ros::Subscriber cmd_vel = n.subscribe("cmd_vel", 100, cmd_velCallback);

  ros::Publisher l_power_pub = n.advertise<std_msgs::Int16>("left_pw", 1000);
  ros::Publisher r_power_pub = n.advertise<std_msgs::Int16>("right_pw", 1000);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(n.ok()){
    
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).toSec();
    //ROS_INFO("dt = %f",dt);
    
    right.delta_pulse = right.pulse_data - right.pre_pulse;
    left.delta_pulse = left.pulse_data - left.pre_pulse;
    //r_delta_pulse = r_delta_pulse == 0 ? 1 : r_delta_pulse;
    //l_delta_pulse = l_delta_pulse == 0 ? 1 : l_delta_pulse;

    //片輪の速度
    right.now_vel = (double)(right.delta_pulse / WHEEL_ONE_SPIN_PULSE) * ((2 * R * M_PI) / dt);
    left.now_vel = (double)(left.delta_pulse / WHEEL_ONE_SPIN_PULSE) * ((2 * R * M_PI) / dt);

    now_vel = (right.now_vel + left.now_vel) / 2;
    now_omega = (right.now_vel - left.now_vel) /  D;
    ROS_INFO("now_vel = %f",now_vel);
    ROS_INFO("now_omega = %f",now_omega);


    right.target_vel = target_vel + target_omega * D/2;
    left.target_vel = target_vel - target_omega * D/2;
   // ROS_INFO("target_omega * D/2 = %f",target_omega * D/2);



    right_pw.data = r_pid_control(right.now_vel, right.target_vel);
    left_pw.data  = l_pid_control(left.now_vel, left.target_vel);
    if(right_pw.data>100)
      right_pw.data = 100;
    else if (right_pw.data<-100)
      right_pw.data = -100;

    if(left_pw.data>100)
      left_pw.data = 100;
    else if (left_pw.data<-100)
      left_pw.data = -100;

    r_power_pub.publish(right_pw);
    l_power_pub.publish(left_pw);

    //ROS_INFO("r_now_vel = %f\tr_target_vel = %f",right.now_vel,right.target_vel);
    //ROS_INFO("l_now_vel = %f\tl_target_vel = %f",left.now_vel,left.target_vel);
    //ROS_INFO("r_pulse = %ld\tr_delta_pulse = %ld",right.pulse_data,right.delta_pulse);
    //ROS_INFO("l_pulse = %ld\tl_delta_pulse = %ld",left.pulse_data,left.delta_pulse);
    
    //save pre data
    last_time = current_time;
    right.pre_pulse = right.pulse_data;
    left.pre_pulse = left.pulse_data;

    ros::spinOnce();   // check for incoming messages
     
    r.sleep();
  }
}


  
