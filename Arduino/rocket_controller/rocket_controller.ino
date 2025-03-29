#include <Wire.h>   // Library for I2wC communication

const int MPU_ADDR = 0x68; // MPU6050 I2C address
int16_t accX, accY, accZ;  // Variables for accelerometer raw data
int16_t gyroX, gyroY, gyroZ; // Variables for the gyro raw data
int16_t temperature; // Variable for the temperature data

char tmp_str[7]; // temprary variable used in convert function


char* convert_int16_to_str(int16_t i){ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Wake up the MPU-6050
  Wire.endTransmission(true);
}

void loop(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // this means that we will start reading from register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // Arduino will send a restart. As a result, the connection is kept active
  Wire.requestFrom(MPU_ADDR, 7*2, true); // Request a total of 14 registers

  // "Wire.read() << 8 | Wire.read();" means that two registers are read and stored in the same variable
  accX = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accX));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accY));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accZ));
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyroX));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyroY));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyroZ));
  Serial.println();

  delay(1000);
}
















// float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
// float roll, pitch, yaw;
// float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
// float elapsedTime, currentTime, previousTime;
// int c = 0;


// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(19200);
//   Wire.begin();                      // Initialize comunication
//   Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
//   Wire.write(0x6B);                  // Talk to the register 6B
//   Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
//   Wire.endTransmission(true);        //end the transmission
//   /*
//   // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
//   Wire.beginTransmission(MPU);
//   Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
//   Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
//   Wire.endTransmission(true);
//   // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
//   Wire.beginTransmission(MPU);
//   Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
//   Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
//   Wire.endTransmission(true);
//   delay(20);
//   */
//   // Call this function if you need to get the IMU error values for your module
//   // calculate_IMU_error();
//   delay(20);  

// }

// void loop() {
//   // === Read acceleromter data === //
//   Wire.beginTransmission(MPU);
//   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//   //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//   AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//   AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//   AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
//   // Calculating Roll and Pitch from the accelerometer data
//   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
//   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
//   // === Read gyroscope data === //
//   previousTime = currentTime;        // Previous time is stored before the actual time read
//   currentTime = millis();            // Current time actual time read
//   elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
//   Wire.beginTransmission(MPU);
//   Wire.write(0x43); // Gyro data first register address 0x43
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
//   GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
//   GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
//   GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
//   // Correct the outputs with the calculated error values
//   GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
//   GyroY = GyroY - 2; // GyroErrorY ~(2)
//   GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
//   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
//   gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
//   gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//   yaw =  yaw + GyroZ * elapsedTime;
//   // Complementary filter - combine acceleromter and gyro angle values
//   roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
//   pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
//   // Print the values on the serial monitor
//   Serial.print(roll);
//   Serial.print("/");
//   Serial.print(pitch);
//   Serial.print("/");
//   Serial.println(yaw);
// }

