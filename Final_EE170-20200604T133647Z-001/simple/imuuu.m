clear a;
a = arduino('COM6', 'Uno', 'Libraries', 'I2C');
imu = mpu9250(a);
accelReadings = readAcceleration(imu)
