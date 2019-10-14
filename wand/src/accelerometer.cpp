#include "accelerometer.h"

MPU6050 accelgyro;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

Motion motion;

void setupAccelerometer() {
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  delay(100);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setFullScaleAccelRange(2);
  accelgyro.setFullScaleGyroRange(2);

  int16_t ax, ay, az, gyro_x, gyro_y, gyro_z;

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Run this code 2000 times
    accelgyro.getMotion6(&ax, &ay, &az, &gyro_x, &gyro_y, &gyro_z);
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
}

Motion *computeMotion() {
  if (!accelgyro.testConnection()) {
    setupAccelerometer();
  }

  int16_t gyro_x, gyro_y, gyro_z;
  accelgyro.getMotion6(&motion.acc_x, &motion.acc_y, &motion.acc_z, &gyro_x, &gyro_y, &gyro_z);
  
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  float restore = .98;
  motion.angle_pitch = restore*(motion.angle_pitch + gyro_x * 0.0000611); 
  motion.angle_roll = restore*(motion.angle_roll + gyro_y * 0.0000611);   
  motion.angle_yaw = restore*(motion.angle_yaw + gyro_z * 0.0000611);
    
  //To dampen the pitch and roll angles a complementary filter is used
  motion.angle_pitch_output = motion.angle_pitch_output * 0.8 + motion.angle_pitch * 0.2;
  motion.angle_roll_output = motion.angle_roll_output * 0.8 + motion.angle_roll * 0.2;   
  motion.angle_yaw_output = motion.angle_yaw_output * 0.8 + motion.angle_yaw * 0.2;

  return &motion;
}
