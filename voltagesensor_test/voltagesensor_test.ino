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

int powerSensors = LOW;
int imuSensor = HIGH;
int gpsSensor = LOW;

//Different Speeds for the thruster motor
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
                          
  Serial.begin(9600);       //initializing Serial Connection for the Transceiver at Hardware Serial 3
  Serial.begin(9600);
  ss.begin(9600);         //initializing software serial connection for the gps module
  pinMode(13, OUTPUT);
  pinMode(analogInput0, INPUT);
  pinMode(analogInput1, INPUT);

                                  
  Serial.setTimeout(10);     //reducing the serial read delay
  delay(5000);
                                  
  ax12a.begin(BaudRate, DirectionPin, &Serial2);  //initializing the AX-12 servo  connected at Serial 2 TX                             
  ax12a.setEndless(ID,ON);      //setting the AX-12 servo into wheelmode
                               
  rudderServo.attach(6);      //initializing the data pin of the rudderServo to the digital pin 6                             
  rudderServo.write(65);    //setting the rudderServo's orientation in the middle                                
  delay(5000);
  propellerMotor.attach(7 );   //initializing the data pin of the propellerMotor to the digital pin 7                                 
  propellerMotor.writeMicroseconds(1500);   //setting the propellerMotor off.
  delay(7000);

  if(imu.begin() != INV_SUCCESS)
    {   
      while(1)
        {
          Serial.println("Unable to communicate with MPU-9250");
          Serial.println("Check connections, and try again");
          Serial.println();
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
  Serial.println("$FEEDBACK,Connected to the USV");
  Serial.println("$FEEDBACK,Welcome to the USV");
  Serial.println("$FEEDBACK,Let's Go and collect trash!");
}

void loop() {
  unsigned long currentMillis = millis();


  if(Serial.available()){
    gStation=Serial.read();
    command=gStation;  
                              //Change the values according to real values of the rudder.  **For the mean time, values below are the default values**
  int turnLeftRudder = 35;     //orientation of the servo when turning left
  
  int centerRudder = 65;      //orientation of the servo when centering
  
  int turnRightRudder = 95  ;  //orientation of the servo when turning right
  
                              //Speed of the conveyor
  int conveyorSpeed0 = 0;
  int conveyorSpeed;
  int conveyorSpeed1 = 300;
  int conveyorSpeed2 = 400;    //**Default Speed when the conveyor is on but you can change the speed if you want**//
  int conveyorSpeed3 = 500;


/////////////////////////////////////////////////////////////////////////////////////////////
  if (command == '0'){
    vSpeed= vSpeed0;
   }
  
  else if (command == '1'){
    vSpeed= vSpeed1;
    }
    
  else if (command == '2'){
    vSpeed= vSpeed2;   
    }
    
  else if (command == '3'){
    vSpeed= vSpeed3;   
  }
  
  else if (command == '4'){
    vSpeed= vSpeed4;     
    }
      
  else if (command == '5'){
    vSpeed= vSpeed5;
//    vSpeed=1540;  
    }
     else if (command == '6'){
    vSpeed= vSpeed6;    
    }
 else if (command == '7'){
    vSpeed= vSpeed7;    
    }
     else if (command == '8'){
    vSpeed= vSpeed8;   
    }
     else if (command == '9'){
    vSpeed= vSpeed9;    
    }
//These commands will activate or deactivate the sensors from sending data to the ground station. '6', '7', '8' is for the powerSensors,imuSensor, gpsSensor to be activated respectively,and will be sending data to the ground station.
// 'B','N','M' is for the powerSensors,imuSensor, gpsSensor to be deactivated respectively,and will stop sending data to the ground station.

  else if (command == 'J'){
    powerSensors= HIGH;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("Current and Voltage Sensor Activated");    
    }
  
  else if (command == 'B'){
    powerSensors= LOW;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("Current and Voltage Sensor deactivated");    
    }
   
   else if (command == 'K'){
    imuSensor= HIGH;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("IMU Sensor Activated");
   } 
   
   else if (command == 'N'){
    imuSensor= LOW;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("IMU Sensor deactivated");
   }
   
   else if (command == 'L'){
    gpsSensor= HIGH;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("GPS Sensor Activated");
   }
   
   else if (command == 'M'){
    gpsSensor= LOW;
//    Seriial.print("$FEEDBACK,");
//    Seriial.println("GPS Sensor deactivated");
    }
   
    
   //**********FORWARD*********//
   if(command=='W'){
        propellerMotor.writeMicroseconds(vSpeed);
//        Seriial.print("$FEEDBACK,");
//        Seriial.println("FORWARD");
        digitalWrite(13, HIGH);   
   }

   //**********STOP*********//
    if(command=='S'){
        propellerMotor.writeMicroseconds(vSpeed0);
//        Seriial.print("$FEEDBACK,");
//        Seriial.println("STOP");
        digitalWrite(13, LOW);   
   }

   //**********LEFT*********// 
   if(command=='A'){
        rudderServo.write(turnLeftRudder);
        delay(10);
//        Seriial.print("$FEEDBACK,");
//        Seriial.println("TURN LEFT");
        digitalWrite(13, HIGH);   
   }

    if(command=='Q'){
        rudderServo.write(centerRudder);
        delay(10);
//        Seriial.print("$FEEDBACK,");
//        Seriial.println("CENTER");
        digitalWrite(13, LOW);   
   }

  //**********RIGHT*********//
   if(command=='D'){
      rudderServo.write(turnRightRudder); 
      delay(10);
//      Seriial.print("$FEEDBACK,");
//      Seriial.println("TURN RIGHT");
      digitalWrite(13, HIGH);   
   }

    if(command=='C'){
      conveyorSpeed = conveyorSpeed2;
      ax12a.turn(ID, RIGHT, conveyorSpeed);
//      Seriial.print("$FEEDBACK,");
//      Seriial.println("CONVEYOR ON");
      digitalWrite(13, HIGH);   
   }

    if(command=='V'){ 
      conveyorSpeed = conveyorSpeed0;
      ax12a.turn(ID, RIGHT, conveyorSpeed);
//      Seriial.print("$FEEDBACK,");
//      Seriial.println("CONVEYOR STOP");
      digitalWrite(13, LOW);   
   }


   delay(10);
 
   
   } 

  //imuSensor
  if ((imuSensor == HIGH) && (currentMillis - previousMillis2 > 500)) {
    previousMillis2 = currentMillis;
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);   
      printIMUData(millis() - pre_ts);
      pre_ts = millis();
    }
  }
//  for gpsSensor



    while (ss.available() > 0 && gpsSensor == HIGH)
            if ((gps.encode(ss.read())) && (currentMillis - previousMillis3 > 1000) ){
              previousMillis3 = currentMillis;
              displayInfo();
            }

}


void powerModuleOutput() {
  value2 = analogRead(analogInput1);
  vout2 = (value2 * 5.0) / 1024.0; // see text
  vin2 = vout2 / (R22/(R12+R22)); 

  adcValue = analogRead(analogInput0);
  adcVoltage = (adcValue / 1024.0) * 5000;
  current = ((adcVoltage - offsetVoltage) / sensitivity);

  if (vin2 < 0.02) {
    vin2 = 0;
  }
  if (current < 0.02) { 
    current = 0;
  }
  Serial.print(String("$VSCS"));
  Serial.print(",");
  Serial.print(String(vin2));
  Serial.print(",");
  Serial.print(String(current));
  Serial.print (",");
  Serial.print(String( "end" ));
  Serial.println();
  }

  
 


void displayInfo()
{ 
  if (gps.location.isUpdated())
  {
    Serial.print(String("$GPS") );
    Serial.print(",");
    Serial.print(gps.location.lat(), 7);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 7);
    Serial.print(F(","));
    Serial.print(gps.speed.kmph());
    Serial.print(F(","));
    Serial.println(String( "end" ));
  }
}

    
void printIMUData(long int dt)
{
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
  heading = imu.computeCompassHeading();
  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch)) + (magY * sin(roll) * sin(pitch)) + (magZ * cos(roll) * sin(pitch));

  yaw =  atan2(Yh, Xh);
  roll = roll* 57.3;
  pitch = pitch* 57.3;
  yaw = yaw*57.3;



  value2 = analogRead(analogInput1);
  vout2 = (value2 * 5.0) / 1023.0; // see text
  vin2 = vout2 / (R22/(R12+R22)); 

  adcValue = analogRead(analogInput0);
  adcVoltage = (adcValue / 1024.0) * 5000;
  current = ((adcVoltage - offsetVoltage) / sensitivity);

  if (vin2 < 0.02) {
    vin2 = 0;
  }
  if (current < 0.02) { 
    current = 0;
  }
    Serial.print(String("$IMU") );
    Serial.print(",");


    Serial.print(String( pitch) );
    Serial.print(",");
    Serial.print(String( roll));
    Serial.print(",");
    Serial.print(String( yaw ));
    Serial.print(",");
    Serial.print(String( heading ));
//    Serial3.print(",");
//    Serial3.print(String(accelX));
//    Serial3.print(",");
//    Serial3.print(String(accelY));
//    Serial3.print(",");
//    Serial3.print(String(accelZ));
//    Serial3.print(",");
//    Serial3.print(String(gyroX));
//    Serial3.print(",");
//    Serial3.print(String(gyroY));
//    Serial3.print(",");
//    Serial3.print(String(gyroZ));
//  Serial3.print(String("$VSCS"));
  Serial.print(",");
  Serial.print(String(vin2));
  Serial.print(",");
  Serial.print(String(current));
  Serial.print (",");
  Serial.print(String( "end" ));
  Serial.println();
  
   
}



