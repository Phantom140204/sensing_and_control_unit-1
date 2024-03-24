#include "communication_interface.hpp"
#include "sensor_interface.hpp"
#include "config.hpp"
#include "thruster_interface.hpp"

const int16_t led_bitmask[9] = { 0x0001, 0x0003, 0x0007,
                                 0x000F, 0x001F, 0x003F,
                                 0x007F, 0x00FF, 0x01FF };

int led_status[9]= {1,1,1,1,1,1,1,1,1};
const uint8_t led_pinmap[9] = {PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8};

geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 angular_velocity;
sensor_msgs::MagneticField magnetic_field;
geometry_msgs::Vector3 orientation;
std_msgs::Int32MultiArray pwm_values;
std_msgs::Float32 depth_data;
std_msgs::Bool calibration;
std_msgs::Int16 led_indicator;

ros::NodeHandle nh;
ros::Publisher linearAccelerationPub("/sensors/linear_acceleration",
                                     &linear_acceleration);
ros::Publisher AngularVelocityPub("/sensors/angular_velocity",
                                  &angular_velocity);
ros::Publisher magneticFieldPub("/sensors/magnetic_field", &magnetic_field);
ros::Publisher OrientationRPYPub("/sensors/orientation", &orientation);
ros::Publisher DepthDataPub("/sensors/depth", &depth_data);
ros::Subscriber<std_msgs::Int32MultiArray> PWMsub("/control/pwm", &throttleCb);
ros::Subscriber<std_msgs::Bool> CalibSub("/control/calibration", &calibrationCb);
ros::Subscriber<std_msgs::Int16> LedSub("/control/led", &ledCb);

void initializeCommunication() {
  nh.initNode();
  nh.subscribe(PWMsub);
  nh.subscribe(CalibSub);
  nh.subscribe(LedSub);
  nh.advertise(linearAccelerationPub);
  nh.advertise(AngularVelocityPub);
  nh.advertise(magneticFieldPub);
  nh.advertise(OrientationRPYPub);
  nh.advertise(DepthDataPub);
}

void sendDepth(float depth) {
  depth_data.data = depth;
  DepthDataPub.publish(&depth_data);
}

void sendIMUReadings(float ax, float ay, float az, float gx, float gy, float gz,
                     float mx, float my, float mz) {
  linear_acceleration.x = ax * G;
  linear_acceleration.y = ay * G;
  linear_acceleration.z = az * G;

  angular_velocity.x = gx;
  angular_velocity.y = gy;
  angular_velocity.z = gz;

  magnetic_field.magnetic_field.x = mx;
  magnetic_field.magnetic_field.y = my;
  magnetic_field.magnetic_field.z = mz;

  linearAccelerationPub.publish(&linear_acceleration);
  AngularVelocityPub.publish(&angular_velocity);
  magneticFieldPub.publish(&magnetic_field);
}

void sendOrientation(float roll, float pitch, float yaw) {
  orientation.x = roll;
  orientation.y = pitch;
  orientation.z = yaw;
  OrientationRPYPub.publish(&orientation);
}

void throttleCb(const std_msgs::Int32MultiArray& pwm_msg) {
  int32_t pwm_values[NUMBER_OF_THRUSTERS];
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS;
       thruster_index++) {
    pwm_values[thruster_index] = pwm_msg.data[thruster_index];
  }
  setThrusterThrottle(pwm_values);
}

void calibrationCb(const std_msgs::Bool& calibration_status) {
  bool calib_status = calibration_status.data;
  if(calib_status)
  {
    callUpdateOffset();
  }
}

void ledCb(const std_msgs::Int16& led_msg) {
  int16_t led_indicator = led_msg.data;
  for (int i=0; i<9; i++)
  {
    if((led_indicator & led_bitmask[i]) == 1)
    led_status[i] = 0;
    digitalWrite(led_pinmap[i],led_status[i]);
  }
}

void checkForCommands() { nh.spinOnce(); }
