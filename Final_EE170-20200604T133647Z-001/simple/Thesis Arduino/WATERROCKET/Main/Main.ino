#include<stdio.h>
#include "TinyGPS.h"
#include<Wire.h>
#include<Servo.h>
//#include<SoftwareSerial.h>
#include<AltSoftSerial.h>
#include<LSM303.h>
//8.247631, 124.250093
//8.247507, 124.249952
//8.247385, 124.250163
#define target_lat 8.247324L
#define target_lon  124.250505L
unsigned long fix_age;
AltSoftSerial SLAVE;
//SoftwareSerial SLAVE(0,13);
LSM303 compass;
TinyGPS gps;
void gpsdump(TinyGPS &gps);
bool feedgps();
void getGPS();
long lat, lon, alt;
float LAT, LON, ALT;
float LATT, LONN, ALTT;
float PREV_ALT = 0, CURR_ALT = 0;
int para = 0;
float ALT_offset = -17.3;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo parachute;
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
double gyroXangle, gyroYangle;
#define degconvert 57.2957786 //there are like 57 degrees in a radian.
signed int val;
int curr;
float ALT_DIFF = 0;
char data[50];
#define servo1start 25 //25
#define servo2start 120 //120
#define servo3start 15 //15
#define servo4start 60 //60

void setup(){
 //GPS.begin(9600);
 SLAVE.begin(9600);
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
  Serial1.begin(9600);
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
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
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroXangle = map(gyroXangle, -180, 180, 180, 0);
  gyroYangle = map(gyroYangle, -180, 180, 180, 0);
  double compAngleX = roll;
  double compAngleY = pitch;
  timer = micros();
  delay(500);
  servo1.attach(14);
  delay(500);
  servo2.attach(6);
  delay(500);
  servo3.attach(9);
  delay(500);
  servo4.attach(10);
  delay(500);
  parachute.attach(11);
  delay(500);
  servo1.write(servo1start);
  delay(500);
  servo2.write(servo2start);
  delay(500);
  servo3.write(servo3start);
  delay(500);
  servo4.write(servo4start);
  delay(500);
  parachute.write(120);
  delay(2000);
  //parachute.write(35);
  //delay(2000);
  //start a timer 
  Serial.println(gyroXangle);
  Serial.println(gyroYangle);
}
void loop(){
 long lat, lon;
 unsigned long fix_age, time, date, speed, course;
 unsigned long chars;
 unsigned short sentences, failed_checksum;
 // retrieves +/- lat/long in 100000ths of a degree
 gps.get_position(&lat, &lon, &fix_age);
 // time in hh:mm:ss, date in dd/mm/yy
 //Serial.println("AAAAAA");
 getGPS();
 //Serial.println("AAAAAA");
 //Serial.print(LAT/100000,7);
 //Serial.print(",");
 Serial1.print(LAT/100000,7);
 Serial1.print(",");
 //Serial.print(LON/100000,7);
 //Serial.print(",");
 Serial1.print(LON/100000,7);
 Serial1.print(",");
 alt = gps.altitude();
 ALT = alt/100.0 + ALT_offset;
 CURR_ALT = ALT;
 if (CURR_ALT < 11) int x = 1;
 /*if (x == 1)*/ ALT_DIFF = PREV_ALT - CURR_ALT;
 if ((ALT_DIFF > 0) && (CURR_ALT < 10000) && (CURR_ALT > 10)
 
 && (para == 0))
 {
   para = 1;
   parachute.write(35);
 }
 PREV_ALT = CURR_ALT;
 //Serial.print(ALT,7);
 //Serial.print(",");
 Serial1.print(ALT,7);
 Serial1.print(","); 
 //Serial.println("AAAAAA");
 compass.read();
 //Serial.println("AAAAAA");
 float heading = compass.heading((LSM303::vector<int>){1, 0, 0});
 //Serial.println(heading);
 //Serial.print(",");
 Serial1.print(heading/degconvert);
 //Serial1.print(0);
 Serial1.print(",");
 Serial.println(compAngleY);
 //Serial.print(",");
 Serial1.print(compAngleY/degconvert);
 Serial1.print(",");
 Serial.println(compAngleX);
 Serial1.println(compAngleX/degconvert);
 //Serial.println("LAT\tLON\tALT\tH\tX\tY");
 double new_compAngleX = map(compAngleX, -180, 180, 180, 0);
 double new_compAngleY = map(compAngleY, -180, 180, 180, 0);
 //Serial.println(servo2start + (new_compAngleX - gyroXangle));
 //Serial.println(servo4start + (new_compAngleX - gyroXangle));
 LATT = LAT/100000;
 LONN = LON/100000;
 if(LATT > 0 && LONN > 0)
 {
  float x =  cos(target_lat/degconvert) * sin(abs((target_lon - LONN)/degconvert));
  float y = cos(LATT/degconvert)*sin(target_lat/degconvert) - sin(LATT/degconvert) * cos(target_lat/degconvert) * cos(abs(target_lon - LONN)/degconvert);
  float bearing = atan2(x, y);
  if (abs(heading - (bearing*degconvert)) > 4)
  {
    if (heading - (bearing*degconvert) > 0)
    {
      servo1.write(servo1start + 70);
      servo3.write(servo3start - 45);
    }
    else if (heading - (bearing*degconvert) < 0)
    {
      servo1.write(servo1start - 70);
      servo3.write(servo3start + 45);
    }
  }
  
  else if (abs(heading - (bearing*degconvert)) < 4)
  {
    servo1.write(servo1start);
    servo3.write(servo3start);  
  }
 }
 //for testing
// servo1.write(servo1start + (360-heading));
// servo3.write(servo3start + (360-heading));
//Serial.println("AAAAAA");

}
void getGPS(){
 bool newdata = false;
 unsigned long start = millis();
 // Every 1 seconds we print an update
 while (millis() - start < 1000)
 {
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
  double new_compAngleX = map(compAngleX, -180, 180, 180, 0);
  double new_compAngleY = map(compAngleY, -180, 180, 180, 0);
  //servo1.write(servo1start + (new_compAngleX - gyroXangle));
  servo2.write(servo2start + (new_compAngleX - gyroXangle));
  //servo3.write(servo3start + (new_compAngleX - gyroXangle));
  servo4.write(servo4start + (new_compAngleX - gyroXangle));
  val = new_compAngleX - gyroXangle;
  SLAVE.write(val);
  //Serial.println(val);
  //Serial.println(val);
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
 while (Serial1.available())
 {
 if (gps.encode(Serial1.read()))
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
