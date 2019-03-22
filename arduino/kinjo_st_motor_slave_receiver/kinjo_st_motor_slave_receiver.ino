#include "kinjo_st_motor_slave_receiver.h"
#include "ti2c_slave.h"
#include <Wire.h>

//下の3つの値を変えるだけで問題ない...はず
uint8_t slave_address = 0x33;
float degree_per_step = 1.8;//ステッピングモータの1ステップあたりの角度単位は°
int rotate_speed = 100;//ステッピングモータの回転スピード　単位はRPM


KinjoStMotorSlave slave = KinjoStMotorSlave(slave_address, degree_per_step, rotate_speed);
void receiveEvent(int howMany)
{
  slave.receive();
  slave.rotation_state = true;
}

void setup() {
  Serial.begin(9600);           // start serial for output
  Wire.onReceive(receiveEvent);
}



void loop() {
  delay(1);
  if (slave.rotation_state == true)
  {
    slave.rotate();
  }
}
