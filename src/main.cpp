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
#include "config.h"

// fuck best practices, im doing this:
using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             BRAIN CONFIGURATION                           */
/*                                                                           */
/*      Port                          Name                        Type       */
/*                                                                           */
/*      1                             lm1                         motor      */
/*      2                             lm2                         motor      */
/*      3                             lm3                         motor      */
/*      8                             rm1                         motor      */
/*      9                             rm2                         motor      */
/*      10                            rm3                         motor      */
/*      4                             intake                      motor      */
/*      5                             claw                        motor      */
/*      19                            distance                    distance   */
/*      18                            inertial                    inertial   */
/*      17                            vision                      vision     */
/*      20                            radio                       radio      */
/*                                                                           */
/*      AB                            lquad                       encoder    */
/*      CD                            rquad                       encoder    */
/*      EF                            bquad                       encoder    */
/*      G                             clamp                       solenoid   */
/*      H                             intakeCyl                   solenoid   */
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