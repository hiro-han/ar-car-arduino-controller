#include <ros.h>
#include <Arduino.h>
#include <Servo.h>
#include "car_control_msgs/CameraControl.h"


const int CAMERA_PIN = 11;

ros::NodeHandle nh;
Servo servo_camera;

void cameraRpyCallback(const car_control_msgs::CameraControl &msg) {
  servo_camera.write(msg.roll + 90);
}

ros::Subscriber<car_control_msgs::CameraControl> sub_camera("/camera_rpy", &cameraRpyCallback);

void setup() {
  servo_camera.attach(CAMERA_PIN);
  servo_camera.write(90);

  nh.initNode();
  nh.subscribe(sub_camera);
}

void loop() {
  nh.spinOnce();
  delay(1);
}