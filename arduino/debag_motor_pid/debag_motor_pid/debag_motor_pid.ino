//23000 pulse / spin (maxon)
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

// main for testing.
uint8_t addr = 0x33;
IseMotorDriver m1 = IseMotorDriver(addr);

int pw = 0;
long enc = 0;

float Kp = 0.05;
float Kd = 0.5;//0.5

float ref_enc = 400;
float err_enc = 0;
float p_err_enc = 0;
float val_pw = 0;

void setup(){
  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");
}

void loop(){
     enc = m1.encorder();
     err_enc = ref_enc - (float)enc;
     val_pw = (err_enc*Kp + (err_enc-p_err_enc)*Kd);
     pw = max(-100,(int)val_pw);
     pw = min(100,(int)pw);
     m1.setSpeed(pw);
     p_err_enc = err_enc;
     Serial.println(pw);
     delay(5);
}
