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

// Assuming global variables are defined and initialized appropriately
double globalPA = 90.0; // Initialized once globally
double curDeg = 0.0;

double prevL = 0.0;
double prevR = 0.0;
double prevS = 0.0;

int odometry(){
  while(true){
    double currentL = lquad.position(degrees);
    double currentR = rquad.position(degrees);
    double currentS = squad.position(degrees);

    double deltaL = currentL - prevL;
    double deltaR = currentR - prevR;
    double deltaS = currentS - prevS;

    double deltaT = (deltaL - deltaR) / (lWheelDist + rWheelDist); 

    double tx = 2 * sin(degreestorad(deltaT)/2) * (deltaS/deltaT + sWheelDist);
    double ty = 2 * sin(degreestorad(deltaT)/2) * (deltaR/deltaT + rWheelDist);

    double deltaPA = atan2(ty, tx); // Use atan2 for better quadrant handling
    double newPA = curDeg + radtodegrees(deltaPA);

    double globDist = sqrt(pow(xPos, 2) + pow(yPos, 2));
    double deltaDist = sqrt(pow(tx, 2) + pow(ty, 2));

    double angleDiff = globalPA - newPA;
    double actualGlobDist = sqrt(pow(globDist, 2) + pow(deltaDist, 2) + (2 * globDist * deltaDist * cos(degreestorad(angleDiff))));

    curDeg += deltaT;

    if ((globDist + (deltaDist * cos(degreestorad(angleDiff)))) >= 0){
      globalPA = asin((deltaDist/actualGlobDist)*sin(degreestorad(angleDiff))); 
    }
    else {
      globalPA = 180 - asin((deltaDist/actualGlobDist)*sin(degreestorad(angleDiff)));
    }

    yPos = actualGlobDist * sin(degreestorad(globalPA));
    xPos = actualGlobDist * cos(degreestorad(globalPA));

    prevL = currentL;
    prevR = currentR;
    prevS = currentS;

    sticks.Screen.clearLine(6);
    sticks.Screen.setCursor(6,0);
    sticks.Screen.print(xPos, yPos)
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