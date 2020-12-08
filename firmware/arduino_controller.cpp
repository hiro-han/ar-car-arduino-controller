#include <ros.h>
#include <Arduino.h>
#include <Servo.h>
#include "car_control_msgs/CameraControl.h"
#include "car_control_msgs/RcCarControl.h"


const int CAMERA_PIN = 11;
const int STEER_PIN = 9;
const int SPEED_PIN = 10;

const int CAMERA_OFFSET = 90; // [degree]
const int STEER_LEFT = 1100; // [micro second]
const int STEER_CENTER = 1500; // [micro second]
const int STEER_RIGHT = 1900; // [micro second]
const int STEER_BACK = 1100; // [micro second]
const int STEER_STOP = 1500; // [micro second]
const int STEER_FRONT = 1900; // [micro second]

ros::NodeHandle nh;
Servo servo_camera;
Servo servo_steer;
Servo servo_speed;


void cameraRpyCallback(const car_control_msgs::CameraControl &msg) {
  servo_camera.write(msg.roll + CAMERA_OFFSET);
}

int calcSteerPulse(const float steer) {
  int pulse;
  if (steer <= -0.1) {
    pulse = (int)((4.0 / 9.0 * steer + 139.0 / 90.0)*1000.0);
    pulse = max(pulse, STEER_LEFT);
  } else if (steer >= 0.1) {
    pulse = (int)((4.0 / 9.0 * steer + 131.0 / 90.0)*1000.0);
    pulse = min(pulse, STEER_RIGHT);
  } else {
    pulse = STEER_CENTER;
  }
  return pulse;
}

int calcSpeedPulse(const float speed) {
  return calcSteerPulse(-1.0 * speed);
}

void carControlCallback(const car_control_msgs::RcCarControl &msg) {
  int steer_pulse = calcSteerPulse(msg.steer);
  int speed_pulse = calcSpeedPulse(msg.speed);
  servo_steer.writeMicroseconds(steer_pulse);
  servo_speed.writeMicroseconds(speed_pulse);
}

ros::Subscriber<car_control_msgs::CameraControl> sub_camera("/camera_rpy", &cameraRpyCallback);
ros::Subscriber<car_control_msgs::RcCarControl> sub_car_control("/car_control", &carControlCallback);

void setup() {
  servo_camera.attach(CAMERA_PIN);
  servo_camera.write(CAMERA_OFFSET);
  servo_steer.attach(STEER_PIN);
  servo_steer.writeMicroseconds(STEER_CENTER);
  servo_speed.attach(SPEED_PIN);
  servo_speed.writeMicroseconds(STEER_STOP);

  nh.initNode();
  nh.subscribe(sub_camera);
  nh.subscribe(sub_camera);
  nh.subscribe(sub_car_control);
}

void loop() {
  nh.spinOnce();
  delay(1);
}