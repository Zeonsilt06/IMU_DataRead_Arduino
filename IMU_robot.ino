//code by : Muhammad Fauzan (Zeonsilt06)

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
float gyro_CM, gyro_PM, elapsedTime; //gyro angle loop parameter
float gyroAngle_x,gyroAngle_y,gyroAngle_z; // variables for gyro angle;
float gyroCal_x, gyroCal_y, gyroCal_z; // variables for gyro calibration
float accelAngleCal_x,accelAngleCal_y; // variables for accelerometer calibration
float accelAngle_x,accelAngle_y; //variables for Accelerometer angle
float pitchFilter, rollFilter;
int cal; // number of n data reading for calibration

void setup() {
  Serial.begin(115200);
  Wire.begin();//begin I2C connection
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
   // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  //calling IMU calibration function before start measuring
  IMU_calibration();
  delay(20);
}
void loop() {
  IMU_data_read();
}

//function for reading IMU data and convert it to Pitch, Yaw, Roll angle
void IMU_data_read(){
  
      //read accelerometer raw data
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      accelerometer_x = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
      
      //calculate accelerometer angle from accelerometer data
      accelAngle_x = atan(accelerometer_x/sqrt(pow(accelerometer_y,2) + pow(accelerometer_z,2)))*180/PI;
      accelAngle_y = atan(accelerometer_y/sqrt(pow(accelerometer_x,2) + pow(accelerometer_z,2)))*180/PI;
      
      //Accelerometer angle correction
      accelAngle_x = accelAngleCal_x - accelAngle_x;
      accelAngle_y = accelAngleCal_y - accelAngle_y;

      //read gyro raw data
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43); // starting with register 0x43 (GYRO_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      gyro_x = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
      
      //gyro data correction  
      gyro_x = gyroCal_x - gyro_x;
      gyro_y = gyroCal_y - gyro_y;
      gyro_z = gyroCal_z - gyro_z;

      //calculate gyro angle from gyro data    
      gyro_PM = gyro_CM;
      gyro_CM = millis();
      elapsedTime = (gyro_CM - gyro_PM)/1000;
      gyroAngle_x = gyroAngle_x + gyro_x*elapsedTime;
      gyroAngle_y = gyroAngle_y + gyro_y*elapsedTime;
      gyroAngle_z = gyroAngle_z + gyro_z*elapsedTime;

      //complementary filter to combine Gyro and Accelerometer data
      pitchFilter = 0.98*gyroAngle_y + 0.02*accelAngle_y;
      rollFilter = 0.98*gyroAngle_x + 0.02*accelAngle_x;

      // Print out data. Uncomment if you want to debug these parameter
//      Serial.print("aX = "); Serial.print(accelerometer_x);
//      Serial.print(" | aY = "); Serial.print(accelerometer_y);
//      Serial.print(" | aZ = "); Serial.print(accelerometer_z);
//      Serial.print(" | gX = "); Serial.print(gyro_x);
//      Serial.print(" | gY = "); Serial.print(gyro_y);
//      Serial.print(" | gZ = "); Serial.print(gyro_z);
      Serial.print(" | Gyro Angle x = "); Serial.print(gyroAngle_x);
      Serial.print(" | Gyro Angle y = "); Serial.print(gyroAngle_y);
      Serial.print(" | Gyro Angle z = "); Serial.print(gyroAngle_z);
//      Serial.print(" | Accelerometer Angle x = "); Serial.print(accelAngle_x);
//      Serial.print(" | Accelerometer Angle y = "); Serial.print(accelAngle_y);
//      Serial.print(" | Pitch = "); Serial.print(pitchFilter);
//      Serial.print(" | Roll = "); Serial.print(rollFilter);
      Serial.println();
}

//function for calibrating IMU raw data
void IMU_calibration(){

      //read accelerometer raw data
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      accelerometer_x = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

            //calculate accelerometer angle from accelerometer data
      accelAngle_x = atan(accelerometer_x/sqrt(pow(accelerometer_y,2) + pow(accelerometer_z,2)))*180/PI;
      accelAngle_y = atan(accelerometer_y/sqrt(pow(accelerometer_x,2) + pow(accelerometer_z,2)))*180/PI;

      //read gyro raw data
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43); // starting with register 0x43 (GYRO_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      gyro_x = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

      //read n number of raw data and sum it
      for (cal=0;cal<2000;cal++){
          gyroCal_x += gyro_x;
          gyroCal_y += gyro_y;
          gyroCal_z += gyro_z;
          accelAngleCal_x += accelAngle_x;
          accelAngleCal_y += accelAngle_y;
      }
      //calculate mean value of n number data reading
      gyroCal_x /= 2000;
      gyroCal_y /= 2000;
      gyroCal_z /= 2000;
      accelAngleCal_x /= 2000;
      accelAngleCal_y /= 2000;
}
