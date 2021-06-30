#include<stdio.h>
#include "TinyGPS.h"
#include<Wire.h>
#include<Servo.h>
#include<SoftwareSerial.h>
unsigned long fix_age;
SoftwareSerial GPS(8,9);
TinyGPS gps;
void gpsdump(TinyGPS &gps);
bool feedgps();
void getGPS();
long lat, lon, alt;
float LAT, LON, ALT;
float LATT, LONN, ALTT;
float ALT_offset = -17.3;
Servo servo1;
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
int val;
int curr;
char data[50];

void setup(){
 GPS.begin(9600);
 Serial.begin(9600);
  // Set up MPU 6050:
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(100);

  //setup starting angle
  //1) collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;
  servo1.attach(5);
  //servo1.write(servo1.read());
  //start a timer
  timer = micros();
}
void loop(){
 long lat, lon;
 unsigned long fix_age, time, date, speed, course;
 unsigned long chars;
 unsigned short sentences, failed_checksum;
 // retrieves +/- lat/long in 100000ths of a degree
 gps.get_position(&lat, &lon, &fix_age);
 // time in hh:mm:ss, date in dd/mm/yy
 getGPS();
 //Serial.print("Latitude : ");
 Serial.print(LAT/100000,7);
 Serial.print(",");
 //Serial.print(" :: Longitude : ");
 Serial.print(LON/100000,7);
 Serial.print(",");
 alt = gps.altitude();
 ALT = alt/100.0 + ALT_offset;
 Serial.println(ALT,7);
 Serial.print(compAngleX);Serial.print("\t");
 Serial.print(compAngleY);Serial.print("\n");
}
void getGPS(){
 bool newdata = false;
 unsigned long start = millis();
 // Every 1 seconds we print an update
 while (millis() - start < 1000)
 {
 //Serial.println("AAAAAA");
 //Now begins the main loop. 
  //Collect raw data from the sensor.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch; 
  val = map(compAngleX, -180, 180, 180, 0);
  servo1.write(val);
 if (feedgps ()){
 newdata = true;
 }
 }
 if (newdata)
 {
 gpsdump(gps);
 }
}
bool feedgps(){
 while (GPS.available())
 {
 if (gps.encode(GPS.read()))
 return true;
 }
 return 0;
}
void gpsdump(TinyGPS &gps)
{
 //byte month, day, hour, minute, second, hundredths;
 gps.get_position(&lat, &lon);
 LAT = lat;
 LON = lon;
 {
 feedgps(); // If we don't feed the gps during this long

 }
}
