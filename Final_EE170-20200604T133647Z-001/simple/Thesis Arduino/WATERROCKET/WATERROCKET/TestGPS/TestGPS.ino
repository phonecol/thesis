/*
    GPS Test

 
 Codes by:
 e-Gizmo Mechatronix Central
 http://www.e-gizmo.com
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 9);

void setup()  
{
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() // run over and over
{
    while(!(mySerial.available())){}
      Serial.write(mySerial.read());
}
