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

motor lm(1, false);
motor rm(8, true);


controller sticks;

encoder lquad = encoder(Thinky.ThreeWirePort.C);
encoder rquad = encoder(Thinky.ThreeWirePort.A);
encoder squad = encoder(Thinky.ThreeWirePort.E);

led re = led(Thinky.ThreeWirePort.G);
led gr = led(Thinky.ThreeWirePort.H);


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  lquad.resetRotation();
  rquad.resetRotation();
  squad.resetRotation();
  xPos=0;
  yPos=0;
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
double degreestoinches(double val){
  double rotations = val/360;
  double inches = rotations*(wheelDiameter*PI);
  return inches;
}
double degreestorad(double val){
  return val*PI/180;
}
double radtodegrees(double val){
  return val*180/PI;
}


int odometry(){
  while(true){
    double currentL = lquad.position(degrees);
    double currentR = rquad.position(degrees);
    double currentS = squad.position(degrees);

    double deltaL = degreestoinches(abs(currentL - prevL));
    if (currentL<prevL) deltaL *= -1;
    double deltaR = degreestoinches(abs(currentR - prevR));
    if (currentR<prevR) deltaR *= -1;
    double deltaS = degreestoinches(abs(currentS - prevS));
    if (currentS<prevS) deltaS *= -1;


    double deltaT = (deltaL - deltaR) / (lWheelDist + rWheelDist);

    double tx = 0, ty = 0;
    if (deltaT == 0){
      tx = deltaS;
       ty = deltaR;
    }else{
      tx = 2 * sin(deltaT)/2 * (deltaS/(deltaT) + sWheelDist);
      ty = 2 * sin(deltaT)/2 * (deltaR/(deltaT) + rWheelDist);
    }

    double r = sqrt(tx*tx + ty*ty);
    double angleA = atan2(ty, tx);
    double angleB = -(curDeg+deltaT/2);

    double deltaX = r*cos(angleA+angleB);
    double deltaY = r*sin(angleA+angleB);

    xPos += deltaX;
    yPos += deltaY;
    curDeg += deltaT;
    if(curDeg < 0) curDeg += 2*PI;
    curDeg = fmod(fmod(curDeg,2*PI) + 2*PI, 2*PI);

    prevL = currentL;
    prevR = currentR;
    prevS = currentS;

    sticks.Screen.clearLine(1);
    sticks.Screen.setCursor(1,0);
    sticks.Screen.print("X position: ");
    sticks.Screen.print(xPos);
    
    sticks.Screen.clearLine(2);
    sticks.Screen.setCursor(2,0);
    sticks.Screen.print("Y position: ");
    sticks.Screen.print(yPos);

     sticks.Screen.clearLine(3);
    sticks.Screen.setCursor(3,0);
    sticks.Screen.print("Current heading: ");
    sticks.Screen.print(radtodegrees(curDeg));
  
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
  sticks.Screen.clearScreen();
  task odom(odometry);

  while (true) {
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);
    // volts gives more power, apparently
    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);

      lm.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
      rm.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);

      if (xPos*xPos + yPos+yPos < 9) re.on();
      else re.off();

      if (radtodegrees(curDeg) < 3 || radtodegrees(curDeg) > 357) gr.on();
      else gr.off();

        
    wait(5, msec); 
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