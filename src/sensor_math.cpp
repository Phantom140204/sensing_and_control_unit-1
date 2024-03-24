#include "sensor_math.hpp"

#include <math.h>
#include <time.h>

#include "Fusion.h"
#include "MS5837.h"
#include "config.hpp"
#include "mpu6050.hpp"

#define SAMPLE_RATE 100

FusionOffset offset;
FusionAhrs ahrs;
FusionEuler euler;
FusionVector gyroscope, accelerometer, magnetometer;
bool start = true;
double current_time = millis(), prev_time = current_time, deltaTime = 0;

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                            0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {-1.21f, 3.58f, -1.02f};

const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
// FusionVector accelerometerOffset = {-0.10, 1.19, -0.04f};
FusionVector accelerometerOffset = {-0.00, -0.11, 0.00};

const FusionMatrix softIronMatrix = {0.979, -0.024, -0.019, -0.024, 0.983,
                                     0.007, -0.019, 0.007,  1.040};
const FusionVector hardIronOffset = {53.31, 92.47, 80.06};

void initializeSensorMath() {
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.5f,
      .gyroscopeRange = 250.0f,
      .accelerationRejection = 20.0f,
      .magneticRejection = 20.0f,
      .recoveryTriggerPeriod = 3 * SAMPLE_RATE,
  };
  FusionAhrsSetSettings(&ahrs, &settings);
}

void updateOffset(MPU6050 gyro) {
  float raw_a[3], raw_g[3], raw_m[3], raw_A[2], raw_G[2];
  for (int sample_no = 0; sample_no < NO_OF_SAMPLES; sample_no++) {
    gyro.getSensorsReadings(raw_A[0], raw_A[1], raw_a[2], raw_G[0], raw_G[1],
                            raw_g[2]);

    raw_a[0] = raw_A[1];
    raw_a[1] = -raw_A[0];
    raw_g[0] = raw_G[1];
    raw_g[1] = -raw_G[0];
    accelerometerOffset.array[0] += raw_a[0] / G;
    accelerometerOffset.array[1] += raw_a[1] / G;
    accelerometerOffset.array[2] += raw_a[2] / G;
    gyroscopeOffset.array[0] += raw_g[0];
    gyroscopeOffset.array[1] += raw_g[1];
    gyroscopeOffset.array[2] += raw_g[2];
  }

  accelerometerOffset.array[0] /= NO_OF_SAMPLES;
  accelerometerOffset.array[1] /= NO_OF_SAMPLES;
  accelerometerOffset.array[2] /= NO_OF_SAMPLES;
  gyroscopeOffset.array[0] /= NO_OF_SAMPLES;
  gyroscopeOffset.array[1] /= NO_OF_SAMPLES;
  gyroscopeOffset.array[2] /= NO_OF_SAMPLES;
  accelerometerOffset.array[2] -= 1;
  // Serial.println("accelerometerOffset: ");
  // Serial.println(accelerometerOffset.array[0]);
  // Serial.println(accelerometerOffset.array[1]);
  // Serial.println(accelerometerOffset.array[2]);
  // Serial.println("gyroscopeOffset: ");
  // Serial.println(gyroscopeOffset.array[0]);
  // Serial.println(gyroscopeOffset.array[1]);
  // Serial.println(gyroscopeOffset.array[2]);
}
void applyIMUCalibration(float &ax, float &ay, float &az, float &gx, float &gy,
                         float &gz, float &mx, float &my, float &mz) {
  gyroscope = {gx, gy, gz};
  accelerometer = {ax, ay, az};
  magnetometer = {mx, my, mz};

  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment,
                                        gyroscopeSensitivity, gyroscopeOffset);
  accelerometer =
      FusionCalibrationInertial(accelerometer, accelerometerMisalignment,
                                accelerometerSensitivity, accelerometerOffset);
  magnetometer =
      FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  ax = accelerometer.axis.x;
  ay = accelerometer.axis.y;
  az = accelerometer.axis.z;

  mx = magnetometer.axis.x;
  my = magnetometer.axis.y;
  mz = magnetometer.axis.z;

  gx = gyroscope.axis.x;
  gy = gyroscope.axis.y;
  gz = gyroscope.axis.z;
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);
}

void updateOrientation(float ax, float ay, float az, float gx, float gy,
                       float gz, float mx, float my, float mz, float &roll,
                       float &pitch, float &yaw) {
  // float _roll, _pitch, _yaw;
  gyroscope = {gx, gy, gz};
  accelerometer = {ax, ay, az};
  magnetometer = {mx, my, mz};
  current_time = millis();
  deltaTime = current_time - prev_time;
  prev_time = current_time;
  if (start) {
    start = !start;
    deltaTime = 0;
  }
  deltaTime = deltaTime / 1000;
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
  const FusionEuler rotation =
      FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  const FusionVector translation = FusionAhrsGetEarthAcceleration(&ahrs);

  roll = rotation.angle.roll;
  pitch = -rotation.angle.pitch;
  yaw = rotation.angle.yaw;
  if (roll < 0) {
    roll += 360;
  }
  if (pitch < 0) {
    pitch += 360;
  }
  if (yaw < 0) {
    yaw += 360;
  }

  // ax=translation.axis.x;
  // ay=translation.axis.y;
  // az=translation.axis.z;
}
