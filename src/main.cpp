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
//#include <string>
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

inertial whee(15);


motor_group leftMotor1(lm1,lm2);
motor_group rightMotor1(rm1,rm2);

motor_group leftMotor(lm1, lm2, lm3);
motor_group rightMotor(rm1, rm2, rm3);
motor arm(12);
motor claw(13);

controller sticks;

encoder lquad = encoder(Thinky.ThreeWirePort.C);
encoder rquad = encoder(Thinky.ThreeWirePort.E);
//encoder bquad = encoder(Thinky.ThreeWirePort.C);


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  lquad.resetRotation();
  rquad.resetRotation();
  whee.setHeading(0, degrees);
  curDeg=0;
}

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, bathroom break etc.
  leftMotor.setStopping(coast);
  rightMotor.setStopping(coast);
  arm.setStopping(hold);
  arm.setMaxTorque(100, percent);
  arm.setVelocity(100, percent);
  claw.setVelocity(100, percent);
}

double inchtodegrees(double val){
  double rotations = val/(4.125*PI);
  double degrees = rotations*360;
  return degrees;
}
double degreestorad(double val){
  return val*PI/180;
}
double radtodegrees(double val){
  return val*180/PI;
}

int odometry(){
  //very rudimentary, just degree tracking right now
  while(true){
    curDeg += ((lquad.position(degrees)-prevL) - (rquad.position(degrees)-prevR)) / (lWheelDist + rWheelDist);
    prevL = lquad.position(degrees);
    prevR = rquad.position(degrees);

    sticks.Screen.clearScreen();
    
    sticks.Screen.print(whee.heading(degrees));
    sticks.Screen.print("\n");
    sticks.Screen.print(curDeg);
    sticks.Screen.setCursor(0,0);
    
    task::sleep(20);
  }
  return 1;
}

int headingPID() {
  while(enableTurnPID) {
    // Heading correction logic
    double headingError = targetDeg - (-whee.rotation(degrees)/2);
        
    // Basic PID for heading correction
    turnTotalError += headingError;
    if (abs(headingError) <.01 || abs(headingError) > 20) turnTotalError = 0;

    double turnDerivative = headingError - turnPrevError;
    turnPrevError = headingError;

    // Adjust heading with PID terms
    double turnPower = headingError * tkP + turnTotalError * tkI + turnDerivative * tkD;

    // Clamp the turn power
    if (turnPower < -12.0) turnPower = -12.0;
    if (turnPower > 12.0) turnPower = 12.0;

    // Use turn power to correct heading while driving
    leftMotor.spin(forward, latpower - turnPower, voltageUnits::volt);
    rightMotor.spin(forward, latpower + turnPower, voltageUnits::volt);

    task::sleep(20);
  }
  return 1;
}

int drivePID(){
  while(enableDrivePID){
    double avgPos = (lquad.position(degrees)+rquad.position(degrees))/2;
    
    error = avgPos-inchtodegrees(driveDist);

    totalError += error;
    if(abs(error)<.01 || abs(error) > 20) totalError=0;

    derivative = error-prevError;
    prevError=error;

    // Calculate drive power (forward movement)
    latpower = error * kP + totalError * kI + derivative * kD;

    if (latpower < -12.0) latpower = -12.0;
    if (latpower > 12.0) latpower = 12.0;

    // Let headingPID function handle heading correction
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
  reset();
  task odom(odometry);
  task dpid(drivePID);
  task hpid(headingPID);
  for(int i=0; i<4; ++i){
  driveDist=96;
  wait(6, seconds);
  enableDrivePID=false;
  targetDeg-=45;
  wait(2, seconds);
  driveDist=0;
  enableDrivePID=true;
  }
  //task tpid(turnPID);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  reset();
  task odom(odometry);
  enableDrivePID = false;
  enableTurnPID=false;

  turnImportance = 0.5;

  transmission = false;
  tlatch = false;

  while (noBitches) {
    
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, apparently
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

    wait(20, msec); 
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
