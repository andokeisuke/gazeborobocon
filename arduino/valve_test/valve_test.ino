#include <ros.h>

#include <custom_msg/valve_msg.h>

const int valveSum = 2;
const int valvePin[valveSum] = {3, 6} ;

void valveOpCB(const custom_msg::valve_msg &data)
{
  bool state = data.state;
  int number = data.valve_number;
  int i;
  if (number == -1)
  {
    for (i = 0; i < valveSum;i++)
    {
      digitalWrite(valvePin[i] , LOW);
    }
  }
  else
  {
    digitalWrite(valvePin[number] , state);
  }
}


ros::NodeHandle  nh;
ros::Subscriber<custom_msg::valve_msg>valve_op_sub("valve_op", &valveOpCB);

void init_valve() {
  int i = 0;
  for (i = 0; i < valveSum; i++)
  {
    pinMode(valvePin[i], OUTPUT);
    digitalWrite(valvePin[i], LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(valve_op_sub);

  init_valve();
}

void loop() {
  nh.getHardware()->setBaud(57600);

  // put your main code here, to run repeatedly:
  delay(5);
  nh.spinOnce();

}
