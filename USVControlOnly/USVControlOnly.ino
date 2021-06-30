#include <Servo.h>
#include "Arduino.h"
#include "AX12A.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <MPU9250_RegisterMap.h>



//assigning values for the AX-12 Servo
#define DirectionPin   12
#define BaudRate      1000000
#define ID        9

#define analogInput0 0
#define analogInput1 1

MPU9250_DMP imu;
double roll , pitch, yaw, heading;
long int pre_ts = 0;

static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

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

int sensitivity = 66;
int adcValue= 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double current = 0;





char             command;
char             gStation;

int vSpeed;
//assigning variable names for the servo
Servo propellerMotor;
Servo rudderServo;


TinyGPSPlus gps;
// The Serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


unsigned long currentTime = millis();
 unsigned long previousMillis1 = 0;
 unsigned long previousMillis2 = 0;
 unsigned long previousMillis3 = 0;

int powerSensors = HIGH;
int imuSensor = HIGH;
int gpsSensor = HIGH;

int vSpeed0=1500;
int vSpeed1=1520;
int vSpeed2=1522;
int vSpeed3=1524;
int vSpeed4=1526;
int vSpeed5=1528;
int vSpeed6=1530;
int vSpeed7=1534;
int vSpeed8=1538;
int vSpeed9=1545;

void setup() {
  // put your setup code here, to run once:

                                 
  Serial3.begin(9600);       //initializing Serial Connection for the Transceiver at Hardware Serial 3
  Serial.begin(9600);
  ss.begin(9600);         //initializing software serial connection for the gps module
  pinMode(13, OUTPUT);
  pinMode(analogInput0, INPUT);
  pinMode(analogInput1, INPUT);

                                  
  Serial3.setTimeout(10);     //reducing the serial read delay
  delay(5000);
                                  
  ax12a.begin(BaudRate, DirectionPin, &Serial2);  //initializing the AX-12 servo                               
  ax12a.setEndless(ID,ON);      //setting the AX-12 servo into wheelmode
                               
  rudderServo.attach(8);      //initializing the data pin of the rudderServo to the digital pin 8                             
  rudderServo.write(60);    //setting the rudderServo's orientation in the middle                                
  delay(5000);
  propellerMotor.attach(9);   //initializing the data pin of the propellerMotor to the digital pin 9                                 
  propellerMotor.writeMicroseconds(1500);   //setting the propellerMotor off.
  delay(7000);

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
  imu.setSampleRate(5);
  imu.setCompassSampleRate(5);
  pre_ts = millis();

                                    //These messages will be sent to the groundStation after a successful set-up//
  Serial3.println("$FEEDBACK,Connected to the USV");
  Serial3.println("$FEEDBACK,Welcome to the USV");
  Serial3.println("$FEEDBACK,Let's Go and collect trash!");
}

void loop() {
  unsigned long currentMillis = millis();


  if(Serial3.available()){
    gStation=Serial3.read();
    command=gStation;  
                              //Change the values according to real values of the rudder.  **For the mean time, values below are the default values**
  int turnLeftRudder = 0;     //orientation of the servo when turning left
  
  int centerRudder = 60;      //orientation of the servo when centering
  
  int turnRightRudder = 120;  //orientation of the servo when turning right
  
                              //Speed of the conveyor
  int conveyorSpeed0 = 0;
  int conveyorSpeed;
  int conveyorSpeed1 = 300;
  int conveyorSpeed2 = 400;    //**Default Speed when the conveyor is on but you can change the speed if you want**//
  int conveyorSpeed3 = 500;


/////////////////////////////////////////////////////////////////////////////////////////////
  if (command == '0'){
    vSpeed= vSpeed0;
//    vSpeed=1500;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 0");
   }
  
  else if (command == '1'){
    vSpeed= vSpeed1;
//    vSpeed=1520;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 1");
    }
    
  else if (command == '2'){
    vSpeed= vSpeed2;
//    vSpeed=1525;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 2");    
    }
    
  else if (command == '3'){
    vSpeed= vSpeed3;
//    vSpeed=1530;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 3");     
  }
  
  else if (command == '4'){
    vSpeed= vSpeed4;
//    vSpeed=1535;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 4");     
    }
      
  else if (command == '5'){
    vSpeed= vSpeed5
//    vSpeed=1540;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 5");     
    }
     else if (command == '6'){
    vSpeed= vSpeed6
//    vSpeed=1540;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 6");     
    }
 else if (command == '7'){
    vSpeed= vSpeed7
//    vSpeed=1540;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 7");     
    }
     else if (command == '8'){
    vSpeed= vSpeed8
//    vSpeed=1540;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 8");     
    }
     else if (command == '9'){
    vSpeed= vSpeed9
//    vSpeed=1540;
    Serial3.print("$FEEDBACK,");
    Serial3.println("SPEED SET TO 9");     
    }
//These commands will activate or deactivate the sensors from sending data to the ground station. '6', '7', '8' is for the powerSensors,imuSensor, gpsSensor to be activated respectively,and will be sending data to the ground station.
// 'B','N','M' is for the powerSensors,imuSensor, gpsSensor to be deactivated respectively,and will stop sending data to the ground station.

  else if (command == '6'){
    powerSensors= HIGH;
    Serial3.print("$FEEDBACK,");
    Serial3.println("Current and Voltage Sensor Activated");    
    }
  
  else if (command == 'B'){
    powerSensors= LOW;
    Serial3.print("$FEEDBACK,");
    Serial3.println("Current and Voltage Sensor deactivated");    
    }
   
   else if (command == '7'){
    imuSensor= HIGH;
    Serial3.print("$FEEDBACK,");
    Serial3.println("IMU Sensor Activated");
   } 
   
   else if (command == 'N'){
    imuSensor= LOW;
    Serial3.print("$FEEDBACK,");
    Serial3.println("IMU Sensor deactivated");
   }
   
   else if (command == '8'){
    gpsSensor= HIGH;
    Serial3.print("$FEEDBACK,");
    Serial3.println("GPS Sensor Activated");
   }
   
   else if (command == 'M'){
    gpsSensor= LOW;
    Serial3.print("$FEEDBACK,");
    Serial3.println("GPS Sensor deactivated");
   }
   
    
   //**********FORWARD*********//
   if(command=='W'){
        propellerMotor.writeMicroseconds(vSpeed);
        Serial3.print("$FEEDBACK,");
        Serial3.println("FORWARD");
        digitalWrite(13, HIGH);   
   }

   //**********STOP*********//
    if(command=='S'){
        propellerMotor.writeMicroseconds(vSpeed0);
        Serial3.print("$FEEDBACK,");
        Serial3.println("STOP");
        digitalWrite(13, LOW);   
   }

   //**********LEFT*********// 
   if(command=='A'){
        rudderServo.write(turnLeftRudder);
        delay(10);
        Serial3.print("$FEEDBACK,");
        Serial3.println("TURN LEFT");
        digitalWrite(13, HIGH);   
   }

    if(command=='Q'){
        rudderServo.write(centerRudder);
        delay(10);
        Serial3.print("$FEEDBACK,");
        Serial3.println("CENTER");
        digitalWrite(13, LOW);   
   }

  //**********RIGHT*********//
   if(command=='D'){
      rudderServo.write(turnRightRudder); 
      delay(10);
      Serial3.print("$FEEDBACK,");
      Serial3.println("TURN RIGHT");
      digitalWrite(13, HIGH);   
   }

    if(command=='C'){
      conveyorSpeed = conveyorSpeed2;
      ax12a.turn(ID, RIGHT, conveyorSpeed);
      Serial3.print("$FEEDBACK,");
      Serial3.println("CONVEYOR ON");
      digitalWrite(13, HIGH);   
   }

    if(command=='V'){ 
      conveyorSpeed = conveyorSpeed0;
      ax12a.turn(ID, RIGHT, conveyorSpeed);
      Serial3.print("$FEEDBACK,");
      Serial3.println("CONVEYOR STOP");
      digitalWrite(13, LOW);   
   }
 
   }
   delay(10);
   //Serial3.println(vSpeed2);
   
   } 



}



