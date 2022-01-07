#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
float gyro_CM, gyro_PM, elapsedTime; //gyro angle loop parameter
float gyroAngle_x,gyroAngle_y,gyroAngle_z; // variables for gyro angle;
float gyroCal_x, gyroCal_y, gyroCal_z;
float accelerometerCal_x,accelerometerCal_y,accelerometerCal_z;
int cal; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
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
  IMU_calibration();
  delay(20);
}
void loop() {
  IMU_data_read();
}

void IMU_data_read(){

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      accelerometer_x = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

      accelerometer_x = accelerometerCal_x - accelerometer_x;
      accelerometer_y = accelerometerCal_y - accelerometer_y;
      accelerometer_z = accelerometerCal_z - accelerometer_z;

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      gyro_x = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

      gyro_x = gyroCal_x - gyro_x;
      gyro_y = gyroCal_y - gyro_y;
      gyro_z = gyroCal_z - gyro_z;
          
      gyro_PM = gyro_CM;
      gyro_CM = millis();
      elapsedTime = (gyro_CM - gyro_PM)/1000;
      gyroAngle_x = gyroAngle_x + gyro_x*elapsedTime;
      gyroAngle_y = gyroAngle_y + gyro_y*elapsedTime;
      gyroAngle_z = gyroAngle_x + gyro_y*elapsedTime;

      // print out data
      Serial.print("aX = "); Serial.print(accelerometer_x);
      Serial.print(" | aY = "); Serial.print(accelerometer_y);
      Serial.print(" | aZ = "); Serial.print(accelerometer_z);
      Serial.print(" | gX = "); Serial.print(gyro_x);
      Serial.print(" | gY = "); Serial.print(gyro_y);
      Serial.print(" | gZ = "); Serial.print(gyro_z);
      Serial.print(" | Gyro Roll = "); Serial.print(gyroAngle_x);
      Serial.print(" | Gyro Pitch = "); Serial.print(gyroAngle_y);
      Serial.print(" | Gyro Yaw = "); Serial.print(gyroAngle_z);
      Serial.println();
}

void IMU_calibration(){
      
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      accelerometer_x = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 6, true); // request a total of 6 registers
      gyro_x = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = (Wire.read()<<8 | Wire.read())/131.0; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
            
      for (cal=0;cal<2000;cal++){
          gyroCal_x += gyro_x;
          gyroCal_y += gyro_y;
          gyroCal_z += gyro_z;
          accelerometerCal_x += accelerometer_x;
          accelerometerCal_y += accelerometer_y;
          accelerometerCal_z += accelerometer_z;
      }
      gyroCal_x /= 2000;
      gyroCal_y /= 2000;
      gyroCal_z /= 2000;
      accelerometerCal_x /= 2000;
      accelerometerCal_y /= 2000;
      accelerometerCal_z /= 2000;
}
