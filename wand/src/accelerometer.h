#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

typedef struct Motion {
  int16_t acc_x, acc_y, acc_z;
  float angle_pitch, angle_roll, angle_yaw;
  int angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;
  float angle_pitch_output, angle_roll_output, angle_yaw_output;
} Motion;

Motion *computeMotion();
void setupAccelerometer();
