/* Purpose: This sketch headMeka_ArduinoROS uses ROS as well as MultiStepper, AccelStepper libraries to control the 
 * head of the Meka. It doesn't include the sensors to check 
 * where is the HOME, but it will be easy to integrate
 * 
 * It gets stucked on the loop, so it would be nice to send little steps
 * through a script in ROS
 *       
 * Author: Enrique Ortega
 * 
 * based on Jesse Weisberg's moveo_ros
 * 
 * #if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
 */

#include <ros.h>


#include <head_meka_control/HeadJoint.h>
#include <std_msgs/Int16.h>
// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.
#include <AccelStepper.h>
#include <MultiStepper.h>
// EG X-Y position bed driven by 2 steppers
#define ENABLE_ALL 8
// Joint 1
#define E1_STEP_PIN        2
#define E1_DIR_PIN         5


// Joint 2
#define Z_STEP_PIN         4
#define Z_DIR_PIN          7
//#define Z_ENABLE_PIN      
//#define Z_MIN_PIN          
//#define Z_MAX_PIN         


AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
MultiStepper steppers;

int joint_step[2];
int joint_status = 0;

int posicion;
long positions[2]; // Array of desired stepper positions



void head_cb(const head_meka_control::HeadJoint& head_steps){
  joint_status = 1;
  joint_step[0] = head_steps.position1;
  joint_step[1] = head_steps.position2;
}




ros::NodeHandle nh;
//std_msgs::Int16 msg;
head_meka_control::HeadJoint msg;

ros::Publisher steps("joint_steps_feedback",&msg);
//instantiate subscribers
ros::Subscriber<head_meka_control::HeadJoint> head_sub("joint_steps",head_cb); //subscribes to joint_steps on head





void setup() {
    pinMode(13,OUTPUT);
    pinMode(ENABLE_ALL, OUTPUT);

    digitalWrite(ENABLE_ALL, LOW);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(head_sub);
   nh.advertise(steps);
  // Configure each stepper
  joint1.setMaxSpeed(500);
  joint1.setAcceleration(150);
  joint2.setMaxSpeed(100);
  joint2.setAcceleration(50);
  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  delay(2000);
  //digitalWrite(13, 1); //toggle led DEACTIVATED TO AVOID ERRORS
}
void loop() {
   if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = joint_step[1]; 

    // Publish back to ros to check if everything's correct
    msg.position1 = positions[0];
    msg.position2 = positions[1];
    //msg.data=positions[0];
    steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
  }
}
