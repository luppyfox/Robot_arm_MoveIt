//#if defined(ARDUINO) && ARDUINO >= 100
//  #include "Arduino.h"
//#else
//  #include <WProgram.h>
//#endif

//#include <ros.h>
//#include <std_msgs/UInt16.h>
//#include <sensor_msgs/JointState.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

ros::NodeHandle  nh;

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();

#define SERVOMIN 150 //150
#define SERVOMAX 600 //600
//#define SERVOCEN (600-150)/2+150
//#define SERVOTEST 300

int num_joint = 5;

int joint2 = 0;
int joint3 = 1;
int joint4 = 2;
int joint5 = 3;
int joint6 = 4;
int joint7 = 5;

double joint2_angle = 375;
double joint3_angle = 375;
double joint4_angle = 375;
double joint5_angle = 375;
double joint6_angle = 375;
double joint7_angle = 375;

double radiansToDegrees(float position_radians)
{
  position_radians = ((position_radians + 1.57) * 143.3121) + SERVOMIN;
  return position_radians;
  //  position_radians = position_radians + 1.6;
  //
  //  return position_radians * 57.2958;

}

void servo_cb(const std_msgs::Float64MultiArray& cmd_msg) {
  //  joint2_angle=radiansToDegrees(cmd_msg.position[0]);
  //  joint3_angle=radiansToDegrees(cmd_msg.position[1]);
  //  joint4_angle=radiansToDegrees(cmd_msg.position[2]);
  //  joint5_angle=radiansToDegrees(cmd_msg.position[3]);
  //  joint6_angle=radiansToDegrees(cmd_msg.position[4]);
  //  joint7_angle=radiansToDegrees(cmd_msg.position[5]);
  //
  //  myServo.setPWM(joint2, 0, joint2_angle);
  //  myServo.setPWM(joint3, 0, joint3_angle);
  //  myServo.setPWM(joint4, 0, joint4_angle);
  //  myServo.setPWM(joint5, 0, joint5_angle);
  //  myServo.setPWM(joint6, 0, joint6_angle);
  //  myServo.setPWM(joint7, 0, joint7_angle);
  digitalWrite(8, LOW);
  delay(2000);
  for (int i = 0; i <= num_joint; i ++ ) {

    myServo.setPWM(i, 0, radiansToDegrees(cmd_msg.data[i]));
    digitalWrite((13 - i), HIGH);
  }
}


ros::Subscriber<std_msgs::Float64MultiArray> sub("/joint_positions", servo_cb);

void setup() {
  //  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  myServo.begin();
  myServo.setPWMFreq(60);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  nh.subscribe(sub);
  delay(10);
  digitalWrite(8, HIGH);
  nh.spinOnce();
}
