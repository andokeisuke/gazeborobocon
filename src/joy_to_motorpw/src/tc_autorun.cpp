#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>

//auto run
#define FOWARD_POWER 15
#define BACK_POWER 0

float l_power, r_power;
long l_pulse, r_pulse;
std_msgs::Int8 l_pw;
std_msgs::Int8 r_pw;

long r_target_pulse = 0;
long l_target_pulse = 0;


void lCallback(const std_msgs::Int32& msg){
  l_target_pulse = msg.data;
  if (l_pulse < l_target_pulse) {
    //foward
    l_power = FOWARD_POWER;
  }
  else if (l_pulse > l_target_pulse) {
    //back
    l_power = BACK_POWER;    
  }
  else if (l_pulse == l_target_pulse) {
    l_power = 0;    
  }  
}

void rCallback(const std_msgs::Int32& msg){
  r_target_pulse = msg.data;
  if (r_pulse < r_target_pulse) {
    //foward
    r_power = FOWARD_POWER;
  }
  else if (r_pulse > r_target_pulse) {
    //back
    r_power = BACK_POWER;    
  }
  else if (r_pulse == r_target_pulse) {
    r_power = 0;    
  }
}

void rPulseCallback(const std_msgs::Int32& msg){
  r_pulse = msg.data;
}
void lPulseCallback(const std_msgs::Int32& msg){
  l_pulse = msg.data;
}

void equal_check() {
  if (r_pulse == r_target_pulse) {
    r_power = 0;    
  }
  if (l_pulse == l_target_pulse) {
    l_power = 0;    
  }  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "autorun");
  ros::NodeHandle n;
  
  ros::Subscriber l_sub = n.subscribe("l_target_pub", 100, lCallback);
  ros::Subscriber r_sub = n.subscribe("r_target_pub", 100, rCallback);    
  ros::Subscriber lp_sub = n.subscribe("l_pulse", 100, lPulseCallback);
  ros::Subscriber rp_sub = n.subscribe("r_pulse", 100, rPulseCallback);    
  ROS_INFO("ok\n");
  ros::Publisher l_power_pub = n.advertise<std_msgs::Int8>("l_power", 1000);
  ros::Publisher r_power_pub = n.advertise<std_msgs::Int8>("r_power", 1000);

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();
    equal_check();
    l_pw.data = l_power;
    r_pw.data = r_power;
    
    l_power_pub.publish(l_pw);
    r_power_pub.publish(r_pw);

   r.sleep();
  }
  
}
