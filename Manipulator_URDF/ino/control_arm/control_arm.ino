#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;
//Servo gripper;
//Servo wrist;
//Servo elbow;
//Servo shoulder;
//Servo base;

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();

#define SERVOMIN 150 //150
#define SERVOMAX 600 //600
//#define SERVOCEN (600-150)/2+150
//#define SERVOTEST 300

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

//double base_angle=90;
//double shoulder_angle=90;
//double elbow_angle=90;
//double wrist_angle=90;

//double prev_base = 0;
//double prev_shoulder = 0;
//double prev_elbow = 0;
//double prev_wrist = 0;

//double prev_joint2 = 0;
//double prev_joint3 = 0;
//double prev_joint4 = 0;
//double prev_joint5 = 0;
//double prev_joint6 = 0;
//double prev_joint7 = 0;

//int gripperState = 0;
//int positionChanged = 0;

double radiansToDegrees(float position_radians)
{
  position_radians = ((position_radians + 1.57)*143.3121)+SERVOMIN;
  return position_radians;
//  position_radians = position_radians + 1.6;
//
//  return position_radians * 57.2958;

}

void servo_cb(const sensor_msgs::JointState& cmd_msg){
//  base_angle=radiansToDegrees(cmd_msg.position[0]);
//  shoulder_angle=radiansToDegrees(cmd_msg.position[1]);
//  elbow_angle=radiansToDegrees(cmd_msg.position[2]);
//  wrist_angle=radiansToDegrees(cmd_msg.position[3]);

  joint2_angle=radiansToDegrees(cmd_msg.position[0]);
  joint3_angle=radiansToDegrees(cmd_msg.position[1]);
  joint4_angle=radiansToDegrees(cmd_msg.position[2]);
  joint5_angle=radiansToDegrees(cmd_msg.position[3]);
  joint6_angle=radiansToDegrees(cmd_msg.position[4]);
  joint7_angle=radiansToDegrees(cmd_msg.position[5]);
  
//  joint2.write(joint2_angle);
//  joint3.write(joint3_angle);
//  joint4.write(joint4_angle);
//  joint5.write(joint5_angle);
//  joint6.write(joint6_angle);
//  joint7.write(joint7_angle);
  
  myServo.setPWM(joint2, 0, joint2_angle);
  myServo.setPWM(joint3, 0, joint3_angle);
  myServo.setPWM(joint4, 0, joint4_angle);
  myServo.setPWM(joint5, 0, joint5_angle);
  myServo.setPWM(joint6, 0, joint6_angle);
  myServo.setPWM(joint7, 0, joint7_angle);
//
//  if (prev_base==base_angle && prev_shoulder==shoulder_angle && prev_elbow==elbow_angle && prev_wrist==wrist_angle && positionChanged==0)
//  {
//    if (gripperState==0)
//    {
//      gripper.write(60);
//      gripperState = 1;
//    }
//    else if (gripperState==1)
//    {
//      gripper.write(0);
//      gripperState = 0;
//    }
//    positionChanged = 1;
//  }
//  else if ((prev_base!=base_angle || prev_shoulder!=shoulder_angle || prev_elbow!=elbow_angle || prev_wrist!=wrist_angle) && positionChanged==1)
//  {
//    positionChanged = 0;
//  }
//
//  prev_base = base_angle;
//  prev_shoulder = shoulder_angle;
//  prev_elbow = elbow_angle;
//  prev_wrist = wrist_angle;
}


ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(115200);
  myServo.begin();
  myServo.setPWMFreq(60);
//  delay(10);

//  base.attach(8);
//  shoulder.attach(9); 
//  elbow.attach(10);
//  wrist.attach(11);
//  gripper.attach(12); 

//  delay(1);
//  base.write(90);
//  shoulder.write(90);
//  elbow.write(90);
//  wrist.write(90);
//  gripper.write(0);
}

void loop(){
  nh.subscribe(sub);
  delay(10);
//  nh.spinOnce();
}
