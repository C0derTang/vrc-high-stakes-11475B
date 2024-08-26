/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       christophertang                                           */
/*    Created:      5/27/2024, 1:32:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

// fuck best practices, im doing this:
using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Thinky;

motor lm1(0, false);
motor lm2(1, false);
motor lm3(2, false);
motor rm1(7, true);
motor rm2(8, true);
motor rm3(9, true);


motor_group leftMotor1(lm1,lm2);
motor_group rightMotor1(rm1,rm2);

motor_group leftMotor(lm1, lm2, lm3);
motor_group rightMotor(rm1, rm2, rm3);
motor arm(12);
motor claw(13);

controller sticks;

encoder lquad = encoder(Thinky.ThreeWirePort.A);
encoder rquad = encoder(Thinky.ThreeWirePort.B);
encoder bquad = encoder(Thinky.ThreeWirePort.C);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  leftMotor.setStopping(coast);
  rightMotor.setStopping(coast);
  arm.setStopping(hold);
  arm.setMaxTorque(100, percent);
  arm.setVelocity(100, percent);
  claw.setVelocity(100, percent);
}

//PID variable tuning (will take fucking forever)
//straight
double kP = 0.5;
double kI = 0.2;
double kD = 0.0;
//turn
double tkP = 0.5;
double tkI = 0.2;
double tkD = 0.0;

// Autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

//lateral PID vals
int error = 0;
int prevError = 0;
int derivative;
int totalError = 0; //integral

//turn PID vals
int turnError = 0;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError = 0; //integral

bool resetDriveSensors = false;

// Important settings!
bool enableDrivePID = true;
bool noBitches = true;

int drivePID(){
  while(enableDrivePID){
    if (resetDriveSensors){
      resetDriveSensors = false;
      leftMotor.setPosition(0, degrees);
      rightMotor.setPosition(0, degrees);
    }

    int leftPos = leftMotor.position(degrees);
    int rightPos = rightMotor.position(degrees);

    // lateral PID //////////////
    int avgPos = (leftPos + rightPos) / 2;

    error = avgPos - desiredValue;
    derivative = error - prevError;
    totalError += error;

    double lateralMotorPower = (error*kP + derivative*kD + totalError*kI);
    /////////////////////////////

    // turn PID /////////////////
    int turnDiff = leftPos - rightPos;

    turnError = turnDiff - desiredTurnValue;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = (turnError*tkP + turnDerivative*tkD + turnTotalError*tkI);
    /////////////////////////////


    leftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    rightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);


    prevError = error;
    turnPrevError = turnError;
    task::sleep(20);
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  task shitstorm(drivePID);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  task::sleep(1000);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  enableDrivePID = false;

  double turnImportance = 0.5;

  bool transmission = false;
  bool tlatch = false;

  while (noBitches) {
    
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, no built in weird PID apparently
    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);
    if (transmission){
      leftMotor1.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rightMotor1.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);
    }else{
      leftMotor.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rightMotor.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);
    }

    if (sticks.ButtonR1.pressing()){
      arm.spin(forward);
    }else if(sticks.ButtonR2.pressing()){
      arm.spin(reverse);
    }else{
      arm.stop();
    }

    if (sticks.ButtonL1.pressing()){
      claw.spin(forward);
    }else if(sticks.ButtonL2.pressing()){
      claw.spin(reverse);
    }else{
      claw.stop();
    }

    if (sticks.ButtonA.pressing()){
      if (!tlatch){
        transmission = !transmission;
        tlatch = true;
      }
    }else{
      tlatch = false;
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
