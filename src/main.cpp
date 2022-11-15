/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\austi                                            */
/*    Created:      Sun Nov 13 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftWheel            motor_group   1, 2            
// RightWheel           motor_group   3, 4            
// IMU                  inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include<bits/stdc++.h>
using namespace vex;
competition Competition;

const double pi = 3.14159;
const double wheelSize = 2.75;

struct PID{
  double KP,KI,KD;
  double error = 0, prevError = 0, totalError = 0;
  int desired_position = 0, motorPower = 0;

  PID(double KP,double KI,double KD){
    this->KP = KP;
    this->KI = KI;
    this->KD = KD;
  }

  void update(int current_position){
    error = desired_position - current_position;
    totalError += error;
    motorPower = error * KP + (error - prevError) * KI + totalError * KD;
    prevError = error;
  }
};

int inchesToDegrees(double inches){
  return inches / (wheelSize * pi) * 360;
}

void init(){
  vexcodeInit();

  LeftWheel.setPosition(0, degrees);
  RightWheel.setPosition(0, degrees);

  LeftWheel.setStopping(brake);
  RightWheel.setStopping(brake);

  LeftWheel.setVelocity(0, percent);
  RightWheel.setVelocity(0, percent);
}

void Drivercontrol(){
  const int deadline = 15;
  int movePower = 0, turnPower = 0;
  double move_decay = 0.95, turn_decay = 0.95;
  
  while(1){
    int axis1 = Controller1.Axis1.position(percent);
    int axis3 = Controller1.Axis3.position(percent);
    if(abs(axis1) <= deadline) axis1 = 0;
    if(abs(axis3) <= deadline) axis3 = 0;

    if(axis3 > 0) movePower = std::max(movePower, axis3);
    if(axis3 < 0) movePower = std::min(movePower, axis3);
    if(!axis3) movePower *= move_decay;

    if(axis1 > 0) turnPower = std::max(turnPower, axis1);
    if(axis1 < 0) turnPower = std::min(turnPower, axis1);
    if(!axis1) turnPower *= turn_decay;

    int maxPower = std::max(100, abs(movePower) + abs(turnPower));
    LeftWheel.setVelocity((movePower + turnPower) * 100 / maxPower, percent);
    RightWheel.setVelocity((movePower - turnPower) * 100 / maxPower, percent);

    LeftWheel.spin(forward);
    RightWheel.spin(forward);
    
    task::sleep(20);
  }
}

bool PID_enable = 1, PID_reset = 1;
PID leftWheelPID(0.3, 0, 0), rightWheelPID(0.3, 0, 0), turnPID(0.3, 0, 0);
double distances = 0, rotations = 0, angles = 0, turnPower = 0;
int speed = 100;

int PID_run(){
  while(PID_enable){
    if(PID_reset){
      PID_reset = 0;
      LeftWheel.setPosition(0, degrees);
      RightWheel.setPosition(0, degrees);
    }

    turnPID.desired_position = angles;
    turnPID.update(IMU.rotation(degrees));
    rotations += turnPID.motorPower;

    leftWheelPID.desired_position = inchesToDegrees(distances) + rotations;
    rightWheelPID.desired_position = inchesToDegrees(distances) - rotations;

    leftWheelPID.update(LeftWheel.position(degrees));
    rightWheelPID.update(RightWheel.position(degrees));

    LeftWheel.setVelocity(leftWheelPID.motorPower * speed / 100, percent);
    RightWheel.setVelocity(rightWheelPID.motorPower * speed / 100, percent);

    LeftWheel.spin(forward);
    RightWheel.spin(forward);

    task::sleep(20);
  }
  return 0;
}

void Autonomous(){
  task PID_task = task(PID_run);
  /*
  distances += 5; // move 5 inches to forward;
  distances -= 5; // move 5 inches to backward;
  angles = 90; // turn to left;
  angles = -90; // turn to right;
  angles += 45; // turn 45 degrees to left side;
  speed = 80; // 80 percent speed
  */
}

int main() {
  init();
  Competition.autonomous(Autonomous);
  Competition.drivercontrol(Drivercontrol);
}

