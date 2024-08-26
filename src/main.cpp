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

// screw best practices, im doing this:
using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Thinky;

//ports are zero indexed!!!

motor lm1(0, true);
motor lm2(1, true);
motor lm3(2, false);

motor rm1(3, true);
motor rm2(4, false);
motor rm3(5, false);

motor int1(6);
motor int2(7, true);
motor_group intake(int1, int2);

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
  lm1.setStopping(coast);
  lm2.setStopping(coast);
  lm3.setStopping(coast);
  rm1.setStopping(coast);
  rm2.setStopping(coast);
  rm3.setStopping(coast);

  intake.setStopping(hold);
  intake.setVelocity(100, percent);
}

void lSpin(double val){
  lm1.spin(forward, val, voltageUnits::volt);
  lm2.spin(forward, val, voltageUnits::volt);
  lm3.spin(forward, val, voltageUnits::volt);
}
void rSpin(double val){
  rm1.spin(forward, val, voltageUnits::volt);
  rm2.spin(forward, val, voltageUnits::volt);
  rm3.spin(forward, val, voltageUnits::volt);
}


//TODO: PID variable tuning (will take forever)
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
      lm1.setPosition(0, degrees);
      rm1.setPosition(0, degrees);
    }

    int leftPos = lm1.position(degrees);
    int rightPos = rm1.position(degrees);

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


    lSpin(lateralMotorPower + turnMotorPower);
    rSpin(lateralMotorPower - turnMotorPower);


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

  while (noBitches) {
    
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, no built in weird PID apparently
    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);

    lSpin(fwdVolts + turnVolts);
    rSpin(fwdVolts - turnVolts);

    if (sticks.ButtonR1.pressing()){
      intake.spin(forward);
    }else if(sticks.ButtonR2.pressing()){
      intake.spin(reverse);
    }else{
      intake.stop();
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