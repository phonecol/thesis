#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "AX12A.h"
#define DirectionPin   12
#define BaudRate      1000000
#define ID        9


static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

#define DHT11_PIN 7
#define analogInput0 0
#define analogInput1 1
#define analogInput2 2

//UltraSonicDistanceSensor distanceSensor(12, 11);  // Initialize sensor that uses digital pins 12 and 11.


TinyGPSPlus gps;
// The Serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

const int QOV = 2.5;


unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 150;

int val_thruster;
int val_rudder;
unsigned long previousTimeLed1 = millis();
long timeIntervalLed1 = 100;
int ledState1 = LOW;
unsigned long previousTimeSerialPrintPotentiometer = millis();
long timeIntervalSerialPrint = 1000;

MPU9250_DMP imu;
Servo myservo;
Servo myservo1;
double roll , pitch, yaw;
long int pre_ts = 0;


float vout2 = 0.0;
float vin2 = 0.0;
float R12 = 30000.0; //
float R22 = 7500.0; //
int value2 = 0;

// Volmeter
float vout3 = 0.0;
float vin3 = 0.0;
float R13 = 30000.0; //
float R23 = 7500.0; //
int value3 = 0;

int offset = 0;

unsigned long lastMillis = 0;



void setup() {
Serial.begin(9600);
Serial.begin(9600);
//2.begin();
ss.begin(9600);

  pinMode(analogInput0, INPUT);
  pinMode(analogInput1, INPUT);

  //para paspas pag read sa serail monitor
  Serial.setTimeout(10);
//  ax12a1.begin(BaudRate, DirectionPin, &Serial1);
  ax12a.begin(BaudRate, DirectionPin, &Serial2);                              //*****FOR CONVEYOR******
  ax12a.setEndless(ID, ON);                                                   //*****FOR CONVEYOR******
  myservo.attach(8); // (pin, min pulse width, max pulse width in microseconds)*****FOR THRUSTER****** 
  myservo1.attach(9);                                                         //*****FOR RUDDER********
  myservo.writeMicroseconds(1000);
  delay(1000);

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(3000);
    }
  }


  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);


  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(10); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(50); // Set mag rate to 10Hz

  pre_ts = millis();

}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available()) {
  //  char comm = Serial.read();
    int command = Serial.parseInt();
    int val = Serial.parseInt();
    int val_thruster = val;
    int val_rudder = val;
    int val_camera = val;
    int val_conveyor = val;
    
 
    //***************************************************************//
    //*****************THRUSTER CONTROL******************************//
    if (command == 1 && val_thruster > 1200 ) {
      myservo.writeMicroseconds(val_thruster);
      Serial.println("woooooooooooooooooooooooooooooooooooo");

    }
    else if ( command == 1 && val_thruster <= 1200) {
      //digitalWrite(ledPin1, LOW);
      myservo.writeMicroseconds(val_thruster);
    }

    //***************************************************************//
    //*****************RUDDER CONTROL******************************//


    else if (command == 2 && val_rudder > 90) {
      myservo1.write(val_rudder);
    }
    else if ( command == 2 && val_rudder < 90) {
      myservo1.write(val_rudder);
    }

    else if ( command == 2 && val_rudder == 90) {
      myservo1.write(val_rudder);
    }






    //***************************************************************//
    //*****************CAMERA ACTUATOR CONTROL******************************//
    else if (command == 3 && val_camera > 90) {


    }
    else if ( command == 3 && val_camera < 90) {
  

    }


    else if ( command == 3 && val_camera == 90) {
  

    }
 //***************************************************************//
    //*****************CONVEYOR CONTROL******************************//
    else if (command == 4) {

      ax12a.turn(ID, RIGHT, val_conveyor);
      Serial.println("woooooooooooooooooooooooooooooooooooo");


    }
  }
//  **********************IMU DATA****************************//
  if (currentTime - previousTimeLed1 > 100) { //timeIntervalLed1

    previousTimeLed1 = currentTime;
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      printIMUData(millis() - pre_ts);
      pre_ts = millis();
    }
  }

  //****************************GPS DATA***************************//
    unsigned long currentMillis = millis();
  
//    if (currentMillis - previousMillis > 1000) {
//      // save the last time you blinked the LED
//      previousMillis = currentMillis;
//   while (ss.available() > 0)
//      if (gps.encode(ss.read()))
//  
//        displayInfo();
//  
//    if (millis() > 5000 && gps.charsProcessed() < 10)
//    {
//      Serial.println(F("No GPS detected: check wiring."));
//      while(true);
//    }
//  }
}


void printIMUData(long int dt)
{

 
  //************************VOLTAGE SENSOR************************//
  value2 = analogRead(analogInput0);
  vout2 = ((value2 * 5.0) / 1024.0) - 0.02; // see text

  float voltage_raw =   (5.0 / 1023.0) * analogRead(analogInput1); // Read the voltage from sensor
  float voltage =  voltage_raw - QOV + 0.012 ;// 0.000 is a value to make voltage zero when there is no current


  //Serial.print(String("$IMU") );
  //  Serial.print(",");
  //  Serial.print(String(accelX));
  //  Serial.print(",");
  //  Serial.print(String(accelY));
  //  Serial.print(",");
  //  Serial.print(String(accelZ));
  //  Serial.print(",");
  //  Serial.print(String(gyroX));
  //  Serial.print(",");
  //  Serial.print(String(gyroY));
  //  Serial.print(",");
  //  Serial.print(String(gyroZ));
  //  Serial.print(",");
  vin2 = vout2 * ((R12 + R22) / R22) ;
  float current = (voltage / 0.185) - 2.81;
  if (vin2 < 0.02) {
    vin2 = 0;
  }
  if (current < 0.02) {
    current = 0;
  }
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx) / 57.3;
  float gyroY = imu.calcGyro(imu.gy) / 57.3;
  float gyroZ = imu.calcGyro(imu.gz) / 57.3;
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
  //Euler angle from accels
  pitch = atan2 (accelY , ( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  roll = atan2(-accelX , ( sqrt((accelY * accelY) + (accelZ * accelZ))));
  // yaw from mag

  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch)) + (magY * sin(roll) * sin(pitch)) + (magZ * cos(roll) * sin(pitch));

  yaw =  atan2(Yh, Xh);
  roll = roll;
  pitch = pitch;
  yaw = yaw;
//  Serial.print(String("$IMU") );
//  Serial.print(",");
  Serial.print(String(vin2));
  Serial.print(",");
  Serial.print(String(current));
  Serial.print (",");
  Serial.print(String( pitch) );
  Serial.print(",");
  Serial.print(String( roll));
  Serial.print(",");
  Serial.println(String( yaw ));
}


void displayInfo()
{

  Serial.print(String("$GPS") );
  Serial.print(",");
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 7);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 7);
    Serial.print(F(","));
    Serial.print(gps.speed.kmph());
    Serial.print(F(","));
  }
  else
  {
    Serial.print(F("INVALID"));
  }



  Serial.println();
}
