#include "Arduino.h"
#include "ti2c_slave.h"
#include <Wire.h>
#define SIZE 100

const char postfix = '$';

Ti2c::Ti2c(uint8_t i2caddr) {
  //initializer
  this->addr = i2caddr;
}

void Ti2c::sendStr(char buf[]) {
  Wire.beginTransmission(this->addr);	 //
  Wire.write(buf);				// 1バイトをキューへ送信
  Wire.write(postfix);
  Wire.endTransmission();		// 送信完了
}

int Ti2c::receiveStr() {
  char buf[100];
  char b_buf[SIZE] = "";
  byte val;
  int i = 0;
  long j = 0;
  while (1) {
    j++;
    if (j > 1000) {
      sprintf(buf, "TimeOut");
      return 0;
    }

    while (Wire.available()) {
      val = Wire.read();
      //Serial.print(val);
      if (val == postfix) {
        b_buf[i++] = '\0';
        break;
      } else {
        b_buf[i++] = val;
      }
    }
    if (val == postfix)
    {
      sprintf(buf, "%s", b_buf);
      return atoi(buf);
    }
  }
}
