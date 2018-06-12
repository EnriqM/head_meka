/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper to controll the head of the meka
 *       
 * Author: Enrique Ortega
 * 
 * based on Jesse Weisberg's moveo_ros package
 * 
 * #if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
 */

#include <ros.h>
#include <head_meka_control/HeadJoint.h>
//#include <std_msgs/Int16.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
// EG X-Y position bed driven by 2 steppers

// Joint 1
#define E1_STEP_PIN        12
#define E1_DIR_PIN         11


// Joint 2
#define Z_STEP_PIN         10
#define Z_DIR_PIN          9
//#define Z_ENABLE_PIN      
//#define Z_MIN_PIN          
//#define Z_MAX_PIN         


AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
MultiStepper steppers;

const int fincarrera1 = 2;     // the number of the pushbutton pi
const int fincarrera2 = 3;
const int ledPin =  13;      // the number of the LED pin
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
bool estado;
bool estado2;

int joint_step[2];
int joint_status = 0;
int posicion;
long positions[2];



void head_cb(const head_meka_control::HeadJoint& head_steps){
  joint_status = 1;
  joint_step[0] = head_steps.position1;
  joint_step[1] = head_steps.position2;
}




ros::NodeHandle nh;
//std_msgs::Int16 msg;
//instantiate subscribers
ros::Subscriber<head_meka_control::HeadJoint> head_sub("joint_steps",head_cb); //subscribes to joint_steps on head
//ros::Publisher steps("joint_steps_feedback",&msg);


void initialjoint1(){

  if (buttonState1 != HIGH){
    joint1.moveTo(-1000);
    estado = true;
  }
  else{
    joint1.moveTo(1000);
    estado = false;
  }
  
   //digitalWrite(ledPin,  buttonState1);
  while (buttonState1 != estado){
      buttonState1 = digitalRead(fincarrera1);
       digitalWrite(ledPin,  buttonState1);
     joint1.run();
  }

  joint1.setCurrentPosition(0); //This will change depending on the angle of the head



 if (estado){ //Esto es para que si se resetea por ROS no de problemas
  joint1.runToNewPosition(30);
 }
 else{
  joint1.runToNewPosition(-30);
 }

}

void initialjoint2(){
  if (buttonState2 != HIGH){
    joint2.moveTo(-1000);
    estado = true;
  }
  else{
    joint2.moveTo(1000);
    estado = false;
  }
   //digitalWrite(ledPin,  buttonState1);
  while (buttonState2 != estado2){
      buttonState2 = digitalRead(fincarrera2);
      joint2.run();
  }
  joint2.setCurrentPosition(0); //This will change depending on the angle of the head
  if (estado2){ //Esto es para que si se resetea por ROS no de problemas
    joint2.runToNewPosition(30);
  }
  else{
   joint2.runToNewPosition(-30);
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(fincarrera1, INPUT);
  pinMode(fincarrera2, INPUT);
 
  buttonState1 = digitalRead(fincarrera1); //Inicial para saber si está pa un lao o pa otro
  buttonState2 = digitalRead(fincarrera2);
  joint_status = 1;

  // Configure each stepper
  joint1.setMaxSpeed(500);
  joint1.setAcceleration(300);
  joint2.setMaxSpeed(500);
  joint2.setAcceleration(300);
  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);

  initialjoint1();
  initialjoint2();
  
  //delay(5000); just to test robustness
  
  nh.initNode();
  nh.subscribe(head_sub);
}


void loop() {
   if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[2];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = joint_step[1]; 

    

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
  }

  // Publish back to ros to check if everything's correct
   // msg.data=positions[0];
    //steps.publish(&msg);
}
