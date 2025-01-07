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

using namespace vex;
using namespace std;

competition Competition;

brain noggin;
controller sticks = controller(primary);

motor frontLeftMotor = motor(PORT1, ratio6_1, false);
motor middleLeftMotor = motor(PORT2, ratio6_1, false);
motor backLeftMotor = motor(PORT3, ratio6_1, false);
motor frontRightMotor = motor(PORT8, ratio6_1, true);
motor middleRightMotor = motor(PORT9, ratio6_1, true);
motor backRightMotor = motor(PORT10, ratio6_1, true);

motor_group leftDrive = motor_group(frontLeftMotor, middleLeftMotor, backLeftMotor);
motor_group rightDrive = motor_group(frontRightMotor, middleRightMotor, backRightMotor);

motor firstStageIntake = motor(PORT5, ratio6_1);
motor secondStageIntake = motor(PORT6, ratio6_1);

motor_group intake = motor_group(firstStageIntake, secondStageIntake);

motor armMotor = motor(PORT5, ratio18_1); // not fs port


triport expander = triport(PORT11);

encoder leftEncoder = encoder(expander.A);
encoder rightEncoder = encoder(expander.E);
encoder backEncoder = encoder(expander.C);

encoder armEncoder = encoder(noggin.ThreeWirePort.A);

digital_out clamp = digital_out(noggin.ThreeWirePort.C);



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  backEncoder.resetRotation();
  globalX=0;
  globalY=0;
}

// Unit Conversions
double inchToDeg(double val){
  double rotations = val/(wheelDiameter*PI);
  double degrees = rotations*360;
  return degrees;
}
double degToInch(double val){
  double rotations = val/360;
  double inches = rotations*(wheelDiameter*PI);
  return inches;
}
double degToRad(double val){
  return val*PI/180;
}
double radToDeg(double val){
  return val*180/PI;
}

// odometry thread
int odometry(){
  while(true){
    double currentL = leftEncoder.position(degrees);
    double currentR = rightEncoder.position(degrees);
    double currentB = backEncoder.position(degrees);

    double deltaL = degToInch(abs(currentL - prevL));
    if (currentL<prevL) deltaL *= -1;
    double deltaR = degToInch(abs(currentR - prevR));
    if (currentR<prevR) deltaR *= -1;
    double deltaB = degToInch(abs(currentB - prevB));
    if (currentB<prevB) deltaB *= -1;

    double deltaT = (deltaL - deltaR) / (leftWheelDist + rightWheelDist);

    double tx = 0, ty = 0;
    if (deltaT == 0){
      tx = deltaB;
       ty = deltaR;
    }else{
      tx = 2 * sin(deltaT)/2 * (deltaB/(deltaT) + backWheelDist);
      ty = 2 * sin(deltaT)/2 * (deltaR/(deltaT) + rightWheelDist);
    }

    double r = sqrt(tx*tx + ty*ty);
    double angleA = atan2(ty, tx);
    double angleB = -(globalHeading+deltaT/2);

    double deltaX = r*cos(angleA+angleB);
    double deltaY = r*sin(angleA+angleB);

    globalX += deltaX;
    globalY += deltaY;
    globalHeading += deltaT;
    if(globalHeading < 0) globalHeading += 2*PI;
    globalHeading = fmod(fmod(globalHeading,2*PI) + 2*PI, 2*PI);

    prevL = currentL;
    prevR = currentR;
    prevB = currentB;

    sticks.Screen.clearLine(1);
    sticks.Screen.setCursor(1,0);
    sticks.Screen.print("X position: ");
    sticks.Screen.print(globalX);
    
    sticks.Screen.clearLine(2);
    sticks.Screen.setCursor(2,0);
    sticks.Screen.print("Y position: ");
    sticks.Screen.print(globalY);

     sticks.Screen.clearLine(3);
    sticks.Screen.setCursor(3,0);
    sticks.Screen.print("Current heading: ");
    sticks.Screen.print(radToDeg(globalHeading));
  
    task::sleep(5);
  }
  return 1;
}

// drive functions
void pointTowardPoint(double targetX, double targetY, bool reverseFacing) {
  double deltaX = targetX - globalX;
  double deltaY = targetY - globalY;

  double desiredHeading = atan2(deltaY, deltaX);

  if (reverseFacing) {
    desiredHeading += PI;
    desiredHeading = fmod(desiredHeading + 2 * PI, 2 * PI); // Normalize to 0-2PI
  }

  double turnError = desiredHeading - globalHeading;

  if (turnError > PI) {
    turnError -= 2 * PI;
  } else if (turnError < -PI) {
    turnError += 2 * PI;
  }

  while (abs(turnError) > degToRad(1)) { // Stop turning when within 1 degree of target
    double turnPower = headingPID.update(globalHeading, desiredHeading);

    // Determine the optimal turn direction
    if (turnError > 0) {
      leftDrive.spin(forward, turnPower, voltageUnits::volt);
      rightDrive.spin(forward, -turnPower, voltageUnits::volt);
    } else {
      leftDrive.spin(forward, -turnPower, voltageUnits::volt);
      rightDrive.spin(forward, turnPower, voltageUnits::volt);
    }

    turnError = desiredHeading - globalHeading;
    if (turnError > PI) {
      turnError -= 2 * PI;
    } else if (turnError < -PI) {
      turnError += 2 * PI;
    }

    wait(5,msec);
  }

  leftDrive.stop();
  rightDrive.stop();
}

void driveToPoint(double targetX, double targetY, bool reverseFacing) {
  pointTowardPoint(targetX, targetY, reverseFacing);

  double deltaX = targetX - globalX;
  double deltaY = targetY - globalY;
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  while (distance > 0.5) { // Stop when within 0.5 inches of the target
    double currentX = globalX;
    double currentY = globalY;

    deltaX = targetX - currentX;
    deltaY = targetY - currentY;
    distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    double desiredHeading = atan2(deltaY, deltaX);

    double turnError = desiredHeading - globalHeading;
    if (turnError > PI) {
      turnError -= 2 * PI;
    } else if (turnError < -PI) {
      turnError += 2 * PI;
    }

    double turnPower = headingPID.update(globalHeading, desiredHeading);
    double drivePower = lateralPID.update(0, distance);

    if (reverseFacing) {
      drivePower *= -1;
    }

    leftDrive.spin(forward, drivePower - turnPower, voltageUnits::volt);
    rightDrive.spin(forward, drivePower + turnPower, voltageUnits::volt);

    wait(5,msec);
  }

  leftDrive.stop();
  rightDrive.stop();
}

void turnarmTo (double targetPosition){
  // TODO: lmfao
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
  intake.setStopping(coast);
  armMotor.setStopping(hold);

  firstStageIntake.setVelocity(100, percent);
  secondStageIntake.setVelocity(66, percent);
  
}

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

    double turnVolts = turnVal * 0.12;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);

    leftDrive.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
    rightDrive.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);

    if (sticks.ButtonL1.pressing()) intake.spin(forward);
    else if (sticks.ButtonL2.pressing()) intake.spin(reverse);
    else intake.stop();

    if (sticks.ButtonR1.pressing()) armMotor.spin(forward);
    else if (sticks.ButtonR2.pressing()) armMotor.spin(reverse);
    else armMotor.stop();

    clampLatch.check(sticks.ButtonX.pressing());
    clamp.set(clampLatch.state);
        
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