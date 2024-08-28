/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       christophertang with help from VP                         */
/*    Created:      5/27/2024, 1:32:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include "vars.h"

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
encoder rquad = encoder(Thinky.ThreeWirePort.C);
encoder bquad = encoder(Thinky.ThreeWirePort.E);


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, bathroom break etc.
  leftMotor.setStopping(coast);
  rightMotor.setStopping(coast);
  leftMotor1.setStopping(coast);
  rightMotor1.setStopping(coast);
  lquad.resetRotation();
  rquad.resetRotation();
  bquad.resetRotation();

}

int drivePID(){
  while(enableDrivePID){
    if (resetDriveSensors){
      resetDriveSensors = false;
      leftMotor.setPosition(0, degrees);
      rightMotor.setPosition(0, degrees);
    }

    int leftPos = leftMotor.position(degrees);
    int rightPos = rightMotor.position(degrees);

    // lateral PID 
    int avgPos = (leftPos + rightPos) / 2;

    error = avgPos - desiredValue;
    derivative = error - prevError;
    totalError += error;

    double lateralMotorPower = (error*kP + derivative*kD + totalError*kI);

    // turn PID 
    int turnDiff = leftPos - rightPos;

    turnError = turnDiff - desiredTurnValue;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = (turnError*tkP + turnDerivative*tkD + turnTotalError*tkI);


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
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  
  enableDrivePID = false;

  turnImportance = 0.5;

  transmission = false;
  tlatch = false;

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

//Attempt at position tracking -- we'll see how it goes

void track(){

  while (noBitches){

    midEncode = bquad.position(degrees);
    leftEncode = lquad.position(degrees);
    rightEncode = rquad.position(degrees);

    leftTrackDis = (leftEncode - prevLeftEncode)/360 * (apple) * (FourOmni);
    rightTrackDis = (rightEncode - prevRightEncode)/360 * (apple) * (FourOmni);
    midTrackDis = (midEncode - prevMidEncode)/360 * (apple) * (TwoOmni);

    prevLeftEncode = leftEncode;
    prevRightEncode = rightEncode;
    prevMidEncode = midEncode;

    d = (leftTrackDis - rightTrackDis)/14.75; //dist between l/r trackers

    changeDirection = d - direction;


    //local offset if moving in straight line
    if (changeDirection = 0){

      posx = midTrackDis;
      posy = rightTrackDis;

    }
    //local offset moving on an arc
    else{

      posx = (2 * sin(changeDirection/2)) * ((midTrackDis/changeDirection) + midTrackDis);
      posy = (2 * sin(changeDirection/2)) * ((rightTrackDis/changeDirection) + rightTrackDis);

    }

    //avg orientation 𝞱m????
    double avgOrient = direction + (changeDirection/2);

    //now we gotta convert to polar, reverse the angle measure, then convert back to rectangular; rotating by -𝞱m
    double polarDeg = sqrt(pow(posx, 2) +pow(posy, 2)) * sin(avgOrient);
    polarDeg = -polarDeg;

  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(track);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}