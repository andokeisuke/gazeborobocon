// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include "ti2c_slave.h"
#include <Wire.h>

#include <Stepper.h>

// change this to the number of steps on your motor
#define STEPS 200

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 8, 9, 10, 11);

// the previous reading from the analog input
int previous = 0;
int input = -1;
int steep;
int s;
int degree = 0;
int pre_degree = 0;


byte dt;
int I2C_address;
Ti2c receiver(0x30);
long receive_pulse;
void debug()
{
  I2C_address = 0x30;
}

void kaiten(int inp);
bool kaiten_state = false;

void receiveEvent(int howMany)
{
  receiver.receiveStr();
 kaiten_state = true;
}

void setup() {
  debug();
  Wire.begin(I2C_address);       // join i2c bus with address #8
  Serial.begin(9600);           // start serial for output
  Serial.println(I2C_address);
  Wire.onReceive(receiveEvent);
  stepper.setSpeed(100);

}



void loop() {
  delay(1);
  if(kaiten_state == true)
  {
    kaiten(atoi(receiver.buf));
  }
}

void kaiten(int tar_degree) {
  Serial.print("oo");
  pre_degree = degree;
  degree = tar_degree - 48;//
  if (degree != pre_degree) {
    s = degree - pre_degree;
    steep =  s / 1.8;
    Serial.println("位置");
    Serial.println(degree);
    Serial.println("step数");
    Serial.println(steep);
  }
  stepper.step(steep);
  kaiten_state = false;
}
