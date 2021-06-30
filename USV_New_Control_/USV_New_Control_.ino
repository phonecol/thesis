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

char             command;
char             gStation;


#define analogInput0 0
#define analogInput1 1
#define analogInput2 2


TinyGPSPlus gps;
// The Serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

MPU9250_DMP imu;
Servo myservo;
Servo myservo1;
double roll , pitch, yaw;
long int pre_ts = 0;

unsigned long previousTimeLed1 = millis();
long timeIntervalLed1 = 100;
int ledState1 = LOW;
unsigned long previousTimeSerialPrintPotentiometer = millis();
long timeIntervalSerialPrint = 1000;
unsigned long currentTime = millis();

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

const int QOV = 2.5;

int offset = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  ss.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(analogInput0, INPUT);
  pinMode(analogInput1, INPUT);

  Serial3.setTimeout(10);

  ax12a.begin(BaudRate, DirectionPin, &Serial2);
  ax12a.setEndless(ID,ON);

  myservo1.attach(8);
  myservo.attach(9);
  myservo.writeMicroseconds(1000);
  myservo1.write(0);
  delay(3000);

  if(imu.begin() != INV_SUCCESS)
    {   
      while(1)
        {
          Serial3.println("Unable to communicate with MPU-9250");
          Serial3.println("Check connections, and try again");
          Serial3.println();
          delay(3000);
        }
    }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(250);
  imu.setAccelFSR(2);
  imu.setLPF(10);
  imu.setSampleRate(3);
  imu.setCompassSampleRate(50);
  pre_ts = millis();



        
    
 Serial3.println("Go!!!");    

}
 int cnt = 0;
unsigned long previousMillis = 0;  

void loop() {
  // put your main code here, to run repeatedly:
 unsigned long starttime = micros();
  receiveCommand();

  requestCommand(); 
    
///////////FOR IMU////////////////////////////////
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      printIMUData(millis() - pre_ts);
      pre_ts = millis();
    }



////////////FOR GPS/////////////////////////////////
    unsigned long currentMillis = millis();
  
    if (currentMillis - previousMillis > 1000) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
         while (ss.available() > 0)
            if (gps.encode(ss.read()))
  
            displayInfo();
  
        if (millis() > 5000 && gps.charsProcessed() < 10)
          {
          Serial3.println(F("No GPS detected: check wiring."));
          while(true);
          }
      }

  
  unsigned long endtime = micros();
//  Serial3.println(endtime-starttime);
  delay(10);
  } 


int poss;
long vSpeed = 1000;  //initial speed set to 1000 = 0 m/s 


void receiveCommand() {
  if(Serial3.available()){
    gStation=Serial3.read();
    command=gStation;
   }
}

void requestCommand() {


    if (command == '0'){
      vSpeed=1000;
      Serial3.print("$FEEDBACK,");
      Serial3.println("SPEED SET TO 0");
      
     
     
     }
    
    else if (command == '1'){
      vSpeed=1200;
      Serial3.print("$FEEDBACK,");
      Serial3.println("SPEED SET TO 1");
      
      }
      
    else if (command == '2'){
      vSpeed=1230;
      Serial3.print("$FEEDBACK,");
      Serial3.println("SPEED SET TO 2");
      
      }
      
    else if (command == '3'){
      vSpeed=1240;
      Serial3.print("$FEEDBACK,");
      Serial3.println("SPEED SET TO 3");
      
    }
    
    else if (command == '4'){
      vSpeed=1250;
      Serial3.print("$FEEDBACK,");
      Serial3.println("SPEED SET TO 4");
      
      }
      
     //**********FORWARD*********//
     if(command=='W'){
         
          myservo.writeMicroseconds(vSpeed);
          Serial3.print("$FEEDBACK,");
          Serial3.println("FORWARD");
          digitalWrite(13, HIGH);   
  

     }

     //**********STOP*********//
     if(command=='S'){
          myservo.writeMicroseconds(1000);
          Serial3.print("$FEEDBACK,");
          Serial3.println("STOP");
          digitalWrite(13, LOW);   
     }

     //**********LEFT*********// 
     if(command=='A'){
      
          myservo1.write(0);
          
          delay(10);
          Serial3.print("$FEEDBACK,");
          Serial3.println("TURN LEFT");
          digitalWrite(13, HIGH);   
     }


      if(command=='Q'){

          myservo1.write(60);
          
          delay(10);
          Serial3.print("$FEEDBACK,");
          Serial3.println("CENTER");
          digitalWrite(13, LOW);   
     }

    //**********RIGHT*********//
     if(command=='D'){

          myservo1.write(120);
          
          delay(10);
          Serial3.print("$FEEDBACK,");
          Serial3.println("TURN RIGHT");
          digitalWrite(13, HIGH);   
     }

     //**********CONVEYOR CONTROL*********// 
     if(command=='C'){

      ax12a.turn(ID, RIGHT, 400);
      Serial3.print("$FEEDBACK,");
      Serial3.println("CONVEYOR ON");
      digitalWrite(13, HIGH);   
     }

      if(command=='V'){

      ax12a.turn(ID, RIGHT, 0);
      Serial3.print("$FEEDBACK,");
      Serial3.println("CONVEYOR STOP");
      digitalWrite(13, LOW);   

      

     }




}


     void printIMUData(long int dt)
{

 
  //************************VOLTAGE SENSOR************************//
  value2 = analogRead(analogInput0);
  vout2 = ((value2 * 5.0) / 1024.0) - 0.02; // see text

  float voltage_raw =   (5.0 / 1023.0) * analogRead(analogInput1); // Read the voltage from sensor
  float voltage =  voltage_raw - QOV + 0.012 ;// 0.000 is a value to make voltage zero when there is no current


  vin2 = vout2 * ((R12 + R22) / R22) ;
  float current = (voltage / 0.185) - 2.81;
  if (vin2 < 0.02) {
    vin2 = 0;
  }
  if (current < 0.02) {
    current = 0;
  }


  //************************IMU SENSOR************************//
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
    Serial3.print(String("$IMU") );
    Serial3.print(",");


    Serial3.print(String( pitch) );
    Serial3.print(",");
    Serial3.print(String( roll));
    Serial3.print(",");
    Serial3.print(String( yaw ));
    Serial3.print(",");
    Serial3.print(String(accelX));
    Serial3.print(",");
    Serial3.print(String(accelY));
    Serial3.print(",");
    Serial3.print(String(accelZ));
    Serial3.print(",");
    Serial3.print(String(gyroX));
    Serial3.print(",");
    Serial3.print(String(gyroY));
    Serial3.print(",");
    Serial3.print(String(gyroZ));
    Serial3.print(",");
    Serial3.print(String(vin2));
    Serial3.print(",");
    Serial3.print(String(current));
    Serial3.print (",");
    Serial3.print(String( "end" ));
    Serial3.println();
    
    
}

void displayInfo()
{

  Serial3.print(String("$GPS") );
  Serial3.print(",");

  if (gps.location.isUpdated())
  {
    Serial3.print(gps.location.lat(), 7);
    Serial3.print(F(","));
    Serial3.print(gps.location.lng(), 7);
    Serial3.print(F(","));
    Serial3.print(gps.speed.kmph());
    Serial3.print(F(","));
  }
  else
  {
      Serial3.print(F("8.2280"));
      Serial3.print(",");
      Serial3.print(F("124.2452"));
      Serial3.print(",");
      Serial3.print(F("3.1"));

  }
//cnt++;
   // Serial3.print(",");
//    Serial3.print(String( "end" ));


  Serial3.println();
 // delay(100);
}

