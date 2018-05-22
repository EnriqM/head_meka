/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
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

// Joint 1
#define E1_STEP_PIN        12
#define E1_DIR_PIN         13


// Joint 2
#define Z_STEP_PIN         11
#define Z_DIR_PIN          10
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
std_msgs::Int16 msg;
//instantiate subscribers
ros::Subscriber<head_meka_control::HeadJoint> head_sub("joint_steps",head_cb); //subscribes to joint_steps on head





void setup() {
    pinMode(13,OUTPUT);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(head_sub);
  // Configure each stepper
  joint1.setMaxSpeed(1000);
  joint1.setAcceleration(300);
  joint2.setMaxSpeed(100);
  joint2.setAcceleration(300);
  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);

  //digitalWrite(13, 1); //toggle led DEACTIVATED TO AVOID ERRORS
}
void loop() {
   if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = joint_step[1]; 

    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
  }
}
