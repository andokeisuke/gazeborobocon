
#define SERVO_PIN 7
#define OPEN_DEG 0
#define CLOSE_DEG 90

#include <Servo.h>

Servo myServo ;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(SERVO_PIN);
}

void loop() {
  myServo.write(OPEN_DEG);
  delay(1000);
  myServo.write(CLOSE_DEG);
  delay(1000);

  
  // put your main code here, to run repeatedly:

}
