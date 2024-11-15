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

motor lm(0, false);
motor rm1(5, true);


controller sticks;

encoder lquad = encoder(Thinky.ThreeWirePort.A);
encoder rquad = encoder(Thinky.ThreeWirePort.B);
encoder squad = encoder(Thinky.ThreeWirePort.C);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  lquad.resetRotation();
  rquad.resetRotation();
  bquad.resetRotation();
}

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, bathroom break etc.
  lm.setStopping(coast);
  rm.setStopping(coast);

  
}

// Unit Conversions
double inchtodegrees(double val){
  double rotations = val/(wheelDiameter*PI);
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
    double deltaL = lquad.position(degrees)-prevL;
    double deltaR = rquad.position(degrees)-prevR;
    double deltaS = squad.position(degrees)-prevS;

    double deltaT = (deltaL - deltaR) / (lWheelDist + rWheelDist);

    curDeg += deltaT;

    double tx = 2 * sin(degreestorad(deltaT)/2) * (deltaS/deltaT + sWheelDist);
    double ty = 2 * sin(degreestorad(deltaT)/2) * (deltaR/deltaT + rWheelDist);

    prevL = lquad.position(degrees);
    prevR = rquad.position(degrees);
    prevS = squad.position(degrees);

    


    sticks.Screen.clearLine(6);
    sticks.Screen.setCursor(6,0);
    sticks.Screen.print(curDeg);
    

    task::sleep(10);
  }
  return 1;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  reset();

  while (true) {
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, apparently
    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);

      lm.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rm.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);

        
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