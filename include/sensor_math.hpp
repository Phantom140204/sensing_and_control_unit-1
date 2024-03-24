#ifndef SENSOR_MATH_HPP
#define SENSOR_MATH_HPP
#include <Arduino.h>
#include "mpu6050.hpp"
#define NO_OF_SAMPLES 3000

void initializeSensorMath();
void updateOffset(MPU6050 gyro);
void applyIMUCalibration(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);
void applyDepthSensorCalibration(float &depth);
void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float &roll, float &pitch, float &yaw);

#endif // SENSOR_MATH_HPP


