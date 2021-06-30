#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>

#include <Wire.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <HCSR04.h>
#include <dht.h>
#include "Arduino.h"
#include "AX12A.h"
#define DirectionPin   12
#define BaudRate      1000000
#define ID        9

#define SerialPort Serial
dht DHT;
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

#define DHT11_PIN 7
#define analogInput0 0
#define analogInput1 1
#define analogInput2 2

//UltraSonicDistanceSensor distanceSensor(12, 11);  // Initialize sensor that uses digital pins 12 and 11.


TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);







const int ledPin1 =  3;// the number of the LED pin
const int ledPin2 =  4;
const int ledPin3 =  5;
const int ledPin4 =  6;
const int ledPin5 =  7;
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
AX12A ax12a1;
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




int val_thruster_on = 1250;
int val_thruster_off =1000;
int val_rudder_right =120;
int val_rudder_left =60;
int val_rudder_center =90;
int val_camera_right =120;
int val_camera_left =60;
int val_camera_center =90;
int val_conveyor_on = 500;        
int val_conveyor_off =0;


void setup() {
  // put your setup code here, to run once:
  Serial3.begin(9600);
  SerialPort.begin(9600);
  ss.begin(9600);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(analogInput0, INPUT);
  pinMode(analogInput1, INPUT);

  //para paspas pag read sa serail monitor
  Serial3.setTimeout(10);
  ax12a1.begin(BaudRate, DirectionPin, &Serial1);
  ax12a.begin(BaudRate, DirectionPin, &Serial2);                              //*****FOR CONVEYOR******
  ax12a.setEndless(ID, ON);                                                   //*****FOR CONVEYOR******
  myservo.attach(10); // (pin, min pulse width, max pulse width in microseconds)*****FOR THRUSTER****** 
  myservo1.attach(9);                                                         //*****FOR RUDDER********
  //myservo.writeMicroseconds(1000);
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
  // put your main code here, to run repeatedly:
  if(Serial3.available()){
       String command = Serial3.readStringUntil('\n');

        if(command.equals("TO")){
//MOVE Forward
//          Serial3.println("Th On");
          Serial3.println(command);
          myservo.writeMicroseconds(val_thruster_on);
        }
        else if(command.equals("TS")){
//          Serial3.println("Th Off");  
          Serial3.println(command);
          myservo.writeMicroseconds(val_thruster_off);
//MOVE Stop
        }
        else if(command.equals("RR")){
//          Serial3.println("->");
          Serial3.println(command);
          ax12a1.move(4, val_rudder_right); //0 degree
          myservo1.write(val_rudder_right);
//TURN Right
        }
        else if(command.equals("RL")){
//          Serial3.println("->");
          Serial3.println(command);
          ax12a1.move(4, val_rudder_left); //0 degree
          myservo1.write(val_rudder_left);
// TURN Left
        }
        else if(command.equals("RC")){
//          Serial3.println("C");
          Serial3.println(command);
          ax12a1.move(4, val_rudder_center); //0 degree
          myservo1.write(val_rudder_center);
//Center Rudder
        }
        else if(command.equals("CR")){
//          Serial3.println("Cam >");
          Serial3.println(command);
          myservo1.write(val_camera_right);
          
//Camera right
        }
        else if(command.equals("CL")){
//          Serial3.println("Cam <");
          Serial3.println(command);
          myservo1.write(val_camera_left);
//Camera left
        }
        else if(command.equals("CC")){
//          Serial3.println("Cam C");
          Serial3.println(command);
          myservo1.write(val_camera_center);
//Camera Center
        }
        else if(command.equals("CO")){
//          Serial3.println("Con On");
          Serial3.println(command);
          ax12a.turn(ID, LEFT, val_conveyor_on);
//Conveyor On
        }
        else if(command.equals("CS")){
//          Serial3.println("Con Off");
          Serial3.println(command);
          ax12a.turn(ID, LEFT, val_conveyor_off);
//Conveyor Stop
        }
  }
//Serial3.println("woiiiii");



if(currentTime - previousTimeLed1 > timeIntervalLed1) {
    
    previousTimeLed1 = currentTime;
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      printIMUData(millis()-pre_ts);
      pre_ts=millis();
    }    
  }
}


         



void printIMUData(long int dt)
{  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx)/57.3;
  float gyroY = imu.calcGyro(imu.gy)/57.3;
  float gyroZ = imu.calcGyro(imu.gz)/57.3;
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);


  //Euler angle from accels
  pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));
  
  // yaw from mag
  
  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));
  
  yaw =  atan2(Yh, Xh);

  
  roll = roll;
  pitch = pitch;
  yaw = yaw;
Serial1.print(String("$IMU") );
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

  Serial3.print(String( pitch) );
  Serial3.print(","); 
  Serial3.print(String( roll));
  Serial3.print(","); 
  Serial3.println(String( yaw ));

}

  


