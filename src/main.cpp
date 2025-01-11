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

motor topLeftMotor = motor(PORT8, ratio18_1, false); //5.5
motor middleLeftMotor = motor(PORT9, ratio6_1, false);
motor bottomLeftMotor = motor(PORT10, ratio6_1, true);
motor topRightMotor = motor(PORT3, ratio18_1, true); //5.5
motor middleRightMotor = motor(PORT2, ratio6_1, true);
motor bottomRightMotor = motor(PORT1, ratio6_1, false);

motor_group leftDrive = motor_group(topLeftMotor, middleLeftMotor, bottomLeftMotor);
motor_group rightDrive = motor_group(topRightMotor, middleRightMotor, bottomRightMotor);

motor firstStageIntake = motor(PORT7, ratio6_1, false);
motor secondStageIntake = motor(PORT6, ratio6_1, false);

motor_group intake = motor_group(firstStageIntake, secondStageIntake);

motor rightArmMotor = motor(PORT4, ratio18_1, true);
motor leftArmMotor = motor(PORT5, ratio18_1,false);

motor_group armMotor = motor_group(rightArmMotor, leftArmMotor);



triport expander = triport(PORT18);

encoder leftEncoder = encoder(expander.A);
encoder rightEncoder = encoder(expander.E);
encoder backEncoder = encoder(expander.C);

encoder armEncoder = encoder(noggin.ThreeWirePort.A);

digital_out clamp = digital_out(noggin.ThreeWirePort.C);

led redLED = led(noggin.ThreeWirePort.F);
led yellowLED = led(noggin.ThreeWirePort.G);
led greenLED = led(noggin.ThreeWirePort.H);

inertial inertialSensor = inertial(PORT19);
vision visionSensor = vision(PORT20);

Toggle clampLatch;

PID lateralPID(.04,.00069,.1);
PID headingPID(.35,.0005,1.8);
PID armPID(0,0,0);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  backEncoder.resetRotation();
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

int debug(){
  while(true){
    /*
   sticks.Screen.clearLine(1);
    sticks.Screen.setCursor(1,0);
    sticks.Screen.print("X position: ");
    sticks.Screen.print(globalX);
    
    sticks.Screen.clearLine(2);
    sticks.Screen.setCursor(2,0);
    sticks.Screen.print("Y position: ");
    sticks.Screen.print(globalY);
    */

     sticks.Screen.clearLine(3);
    sticks.Screen.setCursor(3,0);
    sticks.Screen.print("Current heading: ");
    sticks.Screen.print(radToDeg(globalHeading));
    task::sleep(20);
  }
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
    globalY -= deltaY;
    globalHeading -= deltaT;
    if(globalHeading < 0) globalHeading += 2*PI;
    globalHeading = fmod(fmod(globalHeading,2*PI) + 2*PI, 2*PI);

    prevL = currentL;
    prevR = currentR;
    prevB = currentB; 
  
    task::sleep(5);
  }
  return 1;
}

// drive functions
void turnToHeading(double desiredHeading, double waitTime) {
  double turnError = desiredHeading-radToDeg(globalHeading);

  if (turnError > 180) {
    turnError -= 360;
  } else if (turnError < -180) {
    turnError += 360;
  }

  double turnPower = 0;

  double counter = waitTime/.01;
  while (counter > 0) { 
    turnPower = headingPID.update(0, -turnError, 12);
    
    leftDrive.spin(forward, -turnPower, voltageUnits::volt);
    rightDrive.spin(forward, turnPower, voltageUnits::volt);

    double currentHeading = radToDeg(globalHeading);
    turnError = desiredHeading - currentHeading;

    if (turnError > 180) {
      turnError -= 360;
    } else if (turnError < -180) {
      turnError += 360;
    }
    counter -= 1;
    wait(10, msec);
  }

  leftDrive.stop();
  rightDrive.stop();
}


void pointTowardPoint(double targetX, double targetY, bool reverseFacing) {
  double deltaX = targetX - globalX;
  double deltaY = targetY - globalY;

  double desiredHeading = atan2(deltaY, deltaX);

  if (reverseFacing) {
    desiredHeading += PI;
    desiredHeading = fmod(desiredHeading + 2 * PI, 2 * PI); // Normalize to 0-2PI
  }

  double turnError = radToDeg(desiredHeading) - radToDeg(globalHeading);

  if (turnError > 180) {
    turnError -= 360;
  } else if (turnError < -180) {
    turnError += 360;
  }
  double turnPower =0;

  double counter = abs(turnError)/.55;
  while (counter > 0) {
    turnPower = headingPID.update(0, -turnError, 12);
      leftDrive.spin(forward, -turnPower, voltageUnits::volt);
      rightDrive.spin(forward, turnPower, voltageUnits::volt);

    turnError = radToDeg(desiredHeading) - radToDeg(globalHeading);
      if (turnError > 180) {
    turnError -= 360;
  } else if (turnError < -180) {
    turnError += 360;
  }
    counter -= 1;
    wait(10,msec);
  }

  leftDrive.stop();
  rightDrive.stop();
}

void driveXInches(double distance, double maxPower, double waitTime) {
  double initialPosition = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2;
  double currentDistance = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2-initialPosition;
  double error = inchToDeg(distance)-currentDistance;

  double counter = waitTime/.01;
  while (counter > 0) { // Stop when within 0.5 inches of the target
    currentDistance = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2-initialPosition;
    error = currentDistance-inchToDeg(distance);
    double drivePower = lateralPID.update(0, -error, maxPower);

    leftDrive.spin(forward, drivePower, voltageUnits::volt);
    rightDrive.spin(forward, drivePower, voltageUnits::volt);

    counter -= 1;
    wait(10,msec);
  /* sticks.Screen.clearScreen();
  sticks.Screen.setCursor(1,1);
  sticks.Screen.print(error);*/
  }
  leftDrive.stop();
  rightDrive.stop();
}

void turnarmTo (double targetPosition){
  while(abs(armEncoder.position(degrees)-targetPosition) > 1){
    double power = armPID.update(armEncoder.position(degrees), targetPosition, 6);
    armMotor.spin(forward, power, voltageUnits::volt);
  }
  armMotor.stop();
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

  armMotor.setVelocity(100,percent);
  
}

void redSideAutonomous(void) {
  task odom(odometry);
  driveXInches(20,6,1);
  turnToHeading(330,.5);
  driveXInches(14,6,.6);
  clamp.set(true);
  wait(.2,sec);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  clamp.set(false);
  turnToHeading(-90,.9);
  secondStageIntake.setVelocity(45,percent);
  intake.spin(forward);
  driveXInches(-22,6,1.2); // drive to goal
  intake.stop();
  secondStageIntake.setVelocity(66,percent);
  turnToHeading(0,.9);
  driveXInches(17,4,1);
  clamp.set(true);
  wait(.2,sec);
    driveXInches(-16.75,6,.8);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  armMotor.spinFor(forward, 700, degrees, false);
  turnToHeading(90,1);
  driveXInches(-25,7,3);
  wait(100,sec);
}

void blueSideAutonomous(void) {
  task odom(odometry);
  driveXInches(20,6,1);
  turnToHeading(28,.5);
  
  driveXInches(14,6,.6);
  clamp.set(true);
  wait(.2,sec);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  clamp.set(false);
  turnToHeading(90,.9);
  firstStageIntake.setVelocity(80,percent);
  secondStageIntake.setVelocity(45,percent);
  intake.spin(forward);
  driveXInches(-20,6,1.2); // drive to goal
  intake.stop();
  firstStageIntake.setVelocity(100,percent);
  secondStageIntake.setVelocity(66,percent);
  turnToHeading(-5,.9);
  driveXInches(13.75,4,1);
  clamp.set(true);
  wait(.2,sec);
    driveXInches(-13.5,6,.8);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  armMotor.spinFor(forward, 700, degrees, false);
  turnToHeading(-90,1);
  driveXInches(-30,7,3);
  
  wait(100,sec);
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  clampLatch.state = true;
  reset();
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

    //DEBUG
  

    //END DEBUG
        
    wait(5, msec); 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(blueSideAutonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(10, msec);
  }
}