void setup() {
  Serial.begin(9600);
  pinMode(A2,INPUT_PULLUP);
  // put your setup code here, to run once:

}

void loop() {
  bool state = digitalRead(A2);
  Serial.println(state);
  // put your main code here, to run repeatedly:

}
