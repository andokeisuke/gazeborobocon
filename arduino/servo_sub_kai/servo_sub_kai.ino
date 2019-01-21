//-/Arduino/pubsub/pubsub.inoから改良

#include <ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <servo01/servo_msg.h>
#include <Servo.h>

Servo mySer;

ros::NodeHandle nh;

//std_msgs::String msg;
servo01::servo_msg msg;   
ros::Publisher pub("my_topic3",&msg);
int pin;
int w;





void messageCallback(const servo01::servo_msg& msg){  
   pin=msg.pin; 
  mySer.attach(pin);
  mySer.write(w);
  w=90-w;
  
  pub.publish(msg);
}

//ros::Subscriber<std_msgs::String> sub("my_topic", messageCallback);
ros::Subscriber<servo01::servo_msg> sub("servo_var", messageCallback);

void setup(){
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  //mySer.attach(msg.pin);

  
}

void loop(){
  nh.spinOnce();

  //delay(1000);
  //mySer.write(0);
  
  /*
  if(msg.on_off == true){
    mySer.write(90);
  }else{
    mySer.write(0);
  }
  */
  //delay(1000);
}
