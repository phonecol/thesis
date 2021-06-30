


#include <Servo.h>
Servo myservo;     // create servo object to control the ESC
//int potValue;  // value from the analog pin
int val = 0;
void setup() {
  // Attach the ESC on pin 9
  Serial.begin(9600);
  myservo.attach(9); // (pin, min pulse width, max pulse width in microseconds) 
  myservo.writeMicroseconds(1000);
  delay(7000);
  Serial.println("go");
  
}
void loop() {

  val = Serial.parseInt();
  if(Serial.available() && val != 0){
    if(val == -1)
    {
      myservo.writeMicroseconds(0);
      Serial.println(val);
    }
    else{
      Serial.println(val);
      myservo.writeMicroseconds(val);
    }
  }
} 
