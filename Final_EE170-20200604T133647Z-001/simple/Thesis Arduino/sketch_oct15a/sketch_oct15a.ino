#include <Wire.h>
#include <Servo.h>
#define SerialPort Serial
#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;
double roll , pitch, yaw,pitch1,roll1,yaw1;
long int pre_ts=0;
double pi = 3.1416;
void setup() 
{

    {
  SerialPort.begin(9600);

 if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
     // SerialPort.println("Unable to communicate with MPU-9250");
     // SerialPort.println("Check connections, and try again.");
     // SerialPort.println();
     // delay(3000);
    }
  }

 
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

 
  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(10); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(50); // Set mag rate to 10Hz
}

  pre_ts=millis();
}

//}
void loop() 
{
    if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData(millis()-pre_ts);
    pre_ts=millis();
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

  //SerialPort.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " + String(accelZ) + " g");
  //SerialPort.println("Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + " dps");
  //SerialPort.println("Mag: " + String(magX) + ", " + String(magY) + ", " + String(magZ) + " uT");
  //SerialPort.println("Time: " + String(imu.time) + " ms");

//Euler angle from accel

 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

   // yaw from mag

   float Yh = (magY * cos(roll)) - (magZ * sin(roll));
   float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));

   yaw =  atan2(Yh, Xh);


    roll = roll*57.3;
    pitch = pitch*57.3;
    yaw = yaw*57.3;
    pitch1= pitch * (pi/180);
roll1 = roll * (pi/180);
yaw1 = yaw * (pi/180);
//    Serial.print(String(accelX));
//    Serial.print(",");
//    Serial.print(String(accelY)); 
//    Serial.print(","); 
//    Serial.print(String(accelZ));
//    Serial.print(",");  
//    Serial.print(String(gyroX));
//    Serial.print(",");  
//    Serial.print(String(gyroY)); 
//    Serial.print(","); 
//    Serial.print(String(gyroZ));
//    Serial.print(","); 
  Serial.print(String( pitch) );
  Serial.print(","); 
  Serial.print(String( roll));
  Serial.print(","); 
  Serial.println(String( yaw ));
//delay(2000);

}

