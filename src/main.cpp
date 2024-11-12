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

motor lm1(1, false);
motor lm2(2, false);
motor lm3(0, false);
motor rm1(7, true);
motor rm2(8, true);
motor rm3(9, true);

motor arm(3);
digital_out armp1 = digital_out(Thinky.ThreeWirePort.A);
digital_out armp2 = digital_out(Thinky.ThreeWirePort.C);

inertial whee(10);


limit  armreset  = limit(Thinky.ThreeWirePort.B);


motor_group leftMotor1(lm1,lm2);
motor_group rightMotor1(rm1,rm2);

motor_group leftMotor(lm1, lm2, lm3);
motor_group rightMotor(rm1, rm2, rm3);

motor intake1(5,ratio18_1);
motor intake2(6, ratio18_1, true);
motor_group intake(intake1, intake2);


digital_out clamp = digital_out(Thinky.ThreeWirePort.D);

controller sticks;

encoder lquad = encoder(Thinky.ThreeWirePort.G);
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
void dreset(){
  temprot = lquad.position(degrees);
  driveDist=0;

}

void pre_auton(void) { // in da stripped club, straiht up jorkin' it. and  by it
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, bathroom break etc.
  leftMotor.setStopping(coast);
  rightMotor.setStopping(coast);
  arm.setStopping(hold);
  arm.setMaxTorque(100, percent);
  arm.setVelocity(100, percent);
  intake.setVelocity(100, percent);

  
}

// Unit Conversions
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

    sticks.Screen.clearLine(6);
    sticks.Screen.setCursor(6,0);
    sticks.Screen.print(curdeg);
    

    task::sleep(10);
  }
  return 1;
}

int headingPID() {
  while(enableTurnPID) {
    // Heading correction logic
    double headingError = (curDeg)-targetDeg;
        
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
    leftMotor.spin(forward,(lpower - turnPower), voltageUnits::volt);
    rightMotor.spin(forward, (lpower + turnPower), voltageUnits::volt);
    
    
    task::sleep(5);
  }
  return 1;
}

int ldrivePID(){
  while(true){
    if (!enableDrivePID){
      task::sleep(20);
      continue;
    }
    double lPos = temprot-lquad.position(degrees);
    
    lerror = lPos-inchtodegrees(driveDist);

    ltotalError += lerror;
    if(abs(lerror)<.01 || abs(lerror) > 20) ltotalError=0;

    lderivative = lerror-lprevError;
    lprevError=lerror;

    // Calculate drive power (forward movement)
    lpower = lerror * kP + ltotalError * kI + lderivative * kD;

    if (lpower < -speed) lpower = -speed;
    if (lpower > speed) lpower = speed;
    

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
  dreset();
  curDeg=0;
  
  task odom(odometry);
  task ldpid(ldrivePID);
  task hpid(headingPID);

    dreset();
    speed=10.0;
    driveDist=-11.5;
    wait(1, seconds);
    enableDrivePID=false;
    targetDeg=-30;
    wait(.8,seconds);
    dreset();
    enableDrivePID=true;
    speed=5.0;
    driveDist=-12;
    wait(1.2,seconds);
    clamp.set(true);
    wait(.4,seconds);
    
    intake.spin(reverse);
    wait(1,seconds);
    speed=10.0;
    driveDist = 20;
  
    wait(1,seconds);
    enableDrivePID=false;
    speed=5.0;
    targetDeg = -90;
    wait(1,seconds);
    
    dreset();
    enableDrivePID=true;
    speed=5.0;
    driveDist = 30;
    wait(1.5,seconds);
    driveDist=18;
    wait(1,sec);
    enableDrivePID=false;
    targetDeg=-180;
    wait(.6,sec);
    dreset();
    enableDrivePID=true;
    driveDist=17;
    wait(2,sec);
    driveDist=14;
   wait(1,sec);
    enableDrivePID=false;

ldpid.stop();
hpid.stop();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  reset();
  enableDrivePID = false;
  enableTurnPID=false;
  arm.setPosition(0, turns);

  turnImportance = 0.5;

  while (noBitches) {
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, apparently
    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);
    if (tlatch.state){
      leftMotor1.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rightMotor1.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);
    }else{
      leftMotor.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rightMotor.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);
    }
    clamp.set(clatch.state);
    armp1.set(a1latch.state);
    armp2.set(a2latch.state);
    //armp3.set(!a2latch.state);

    if (sticks.ButtonL1.pressing()){
      intake.spin(forward);
    }else if(sticks.ButtonL2.pressing()){
      intake.spin(reverse);
    }else{
      intake.stop();
    }


    if (arm.position(degrees) < 700 && sticks.ButtonR1.pressing()){
      arm.spin(forward);
    }else if (armreset.pressing()){
      arm.setPosition(0,degrees);
      arm.stop();
    }else if(sticks.ButtonR2.pressing()){
      arm.spin(reverse);
    }else{
      arm.stop();
    }

    //tlatch.check(sticks.ButtonA.pressing()); // transmission
    clatch.check(sticks.ButtonX.pressing()); // clamp
    a1latch.check(sticks.ButtonB.pressing()); // first arm pistons
    a2latch.check(sticks.ButtonY.pressing()); // 2nd arm pistons

    
    wait(10, msec); 
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
    wait(10, msec);
  }
}