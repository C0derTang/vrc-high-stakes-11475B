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

// 8 and 18 are fried
motor topLeftMotor = motor(PORT16, ratio18_1, false); //5.5
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



triport expander = triport(PORT15);

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

Toggle clampLatch(2);
Toggle armLatch(3);

PID lateralPID(.04,.00069,.1);
PID headingPID(.35,.0005,1.8);
PID armPID(.14,0,8);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void reset(){
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  backEncoder.resetRotation();
  armEncoder.resetRotation();
  clamp.set(false);

  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
  intake.setStopping(brake);
  armMotor.setStopping(hold);

  //120 * 5

  firstStageIntake.setVelocity(100, percent);
  secondStageIntake.setVelocity(100, percent);
  intake.setMaxTorque(100,percent);

  armMotor.setVelocity(100,percent);
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

int headingPID(){
  while(true){
    double error = desiredHeading-radToDeg(globalHeading);
    if (turnError > 180) {
      turnError -= 360;
    } else if (turnError < -180) {
      turnError += 360;
    }
    
    turnPower = headingPID.update(0, -error, 12);
    
    leftDrive.spin(forward, -turnPower, voltageUnits::volt);
    rightDrive.spin(forward, turnPower, voltageUnits::volt);


    task::sleep(5);
  }
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

void driveXInches(double distance, double maxPower, double waitTime) {
  double initialPosition = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2;
  double currentDistance = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2-initialPosition;
  double error = inchToDeg(distance)-currentDistance;

  double initialHeading = radToDeg(globalHeading);

  double counter = waitTime/.01;
  while (counter > 0) { // Stop when within 0.5 inches of the target
    currentDistance = -(leftEncoder.position(degrees) + rightEncoder.position(degrees))/2-initialPosition;
    error = currentDistance-inchToDeg(distance);
    double drivePower = lateralPID.update(0, -error, maxPower);

    double headingError = initialHeading - radToDeg(globalHeading);
    if (headingError > 180) {
      headingError -= 360;
    } else if (headingError < -180) {
      headingError += 360;
    }

    double correctionPower = headingPID.update(0, -headingError, 1);

    leftDrive.spin(forward, drivePower - correctionPower, voltageUnits::volt);
    rightDrive.spin(forward, drivePower + correctionPower, voltageUnits::volt);

    counter -= 1;
    wait(10,msec);
  /* sticks.Screen.clearScreen();
  sticks.Screen.setCursor(1,1);
  sticks.Screen.print(error);*/
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

  turnToHeading(radToDeg(desiredHeading), 2);
}

void driveToPoint(double targetX, double targetY, double offset, bool reverseFacing){
  pointTowardPoint(targetX, targetY, backwards);
  deltaX = targetX - globalX;
  deltaY = targetY - globalY;
  travelDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
  if (reverseFacing) travelDistance *= -1;
  driveXInches(travelDistance)
}



int armControlThread(){
  while(true){ 
    double power = armPID.update(armEncoder.position(degrees), targetArmPosition, 12);
    armMotor.spin(forward, power, voltageUnits::volt);

    task::sleep(1);
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
  intake.setStopping(brake);
  armMotor.setStopping(hold);

  //120 * 5

  firstStageIntake.setVelocity(100, percent);
  secondStageIntake.setVelocity(100, percent);
  intake.setMaxTorque(100,percent);

  armMotor.setVelocity(100,percent);
  
}

void redCloseSideAutonomous(void) {
  reset();
  task odom(odometry);
  driveXInches(19.5,6,1);
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
  turnToHeading(355,.9);
  driveXInches(16,4,1);
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

void redFarSideAutonomous(void) {
  reset();
  task odom(odometry);
  driveXInches(19.5,6,1);
  turnToHeading(28,.5);
  
  driveXInches(14,6,.6);
  clamp.set(true);
  wait(.2,sec);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  turnToHeading(90,1);
  intake.spin(forward);
  driveXInches(-20,6,1.5); // drive to goal
  turnToHeading(175,1);
  driveXInches(-10,4,1);
  wait(.5,sec);
  driveXInches(10,6,.8);  
  wait(.5,sec);
  intake.stop();
  turnToHeading(250, 1.3);
  armMotor.spinFor(forward, 700, degrees, false);
  driveXInches(-22,6,3);  

}



void blueCloseSideAutonomous(void) {
  reset();
  task odom(odometry);
  driveXInches(20.5,6,1);
  turnToHeading(28,.5);
  
  driveXInches(14,6,.6);
  clamp.set(true);
  wait(.2,sec);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  clamp.set(false);
  turnToHeading(90,.9);
  secondStageIntake.setVelocity(48,percent);
  intake.spin(forward);
  driveXInches(-20,6,1.2); // drive to goal
  intake.stop();
  firstStageIntake.setVelocity(100,percent);
  secondStageIntake.setVelocity(66,percent);
  turnToHeading(-5,.9);
  driveXInches(13.5,4,1);
  clamp.set(true);
  wait(.2,sec);
    driveXInches(-13.5,6,.8);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  armMotor.spinFor(forward, 700, degrees, false);
  turnToHeading(-90,1);
  driveXInches(-26,7,3);
  
  wait(100,sec);
}

void blueFarSideAutonomous(void) {
    reset();
  task odom(odometry);
  driveXInches(20.5,6,1);
  turnToHeading(332,.5);
  
  driveXInches(14,6,.6);
  clamp.set(true);
  wait(.2,sec);
  intake.spin(forward);
  wait(1,sec);
  intake.stop();
  turnToHeading(270,1);
  intake.spin(forward);
  driveXInches(-21,6,1.2); // drive to goal
  driveXInches(1,8,.1); // drive to goal
  turnToHeading(185,1);
  driveXInches(-10,5,1);
  wait(.5,sec);
  driveXInches(10,6,.8);  
  wait(.5,sec);
  intake.stop();
  turnToHeading(110, 1.3);
  armMotor.spinFor(forward, 700, degrees, false);
  driveXInches(-22,6,3);  

}


void skills(void){
  reset();
  task odom(odometry);
  intake.setVelocity(100, percent);
  intake.spin(forward);
  wait(.6,sec);
  intake.stop();
  intake.setVelocity(75,percent);
  driveXInches(-13, 6, 1.2);
  turnToHeading(86,1);
  driveXInches(22,6,.9);
  clamp.set(true);
  wait(.2,sec);
  driveXInches(-4,6,.1);
  intake.spin(forward);
  turnToHeading(353,1);
  driveXInches(-24, 6, 2);
  turnToHeading(270,1);
  driveXInches(-25, 6, 2);
  turnToHeading(185, 1.5);//
  driveXInches(-20, 6, 2);
  driveXInches(-8, 6, 1);
  driveXInches(6, 6, .5);
  turnToHeading(270,1);
  driveXInches(-12, 6, .5);
  turnToHeading(4,1);
  clamp.set(false);
  driveXInches(10, 6, 1);
  driveXInches(-10, 6, 1);
  //new
  turnToHeading(270,1);
  driveXInches(100, 6, 5);
  clamp.set(true);
  turnToHeading(330, 1);
  driveXInches(14, 6, 1.2);
  clamp.set(false);
  driveXInches(-14, 6, 1.2);

  intake.stop();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  task arm(armControlThread);

  while (true) {
    double turnVal = sticks.Axis1.position(percent);
    double fwdVal = sticks.Axis3.position(percent);

    double turnVolts = turnVal * 0.11;
    double fwdVolts = fwdVal * 0.12 * (1-(abs(turnVolts/12.0)) * turnImportance);

    leftDrive.spin(forward, fwdVolts + turnVolts, voltageUnits::volt);
    rightDrive.spin(forward, fwdVolts - turnVolts, voltageUnits::volt);

    if (sticks.ButtonL1.pressing()) intake.spin(forward);
    else if (sticks.ButtonL2.pressing()) intake.spin(reverse);
    else if (!reverseIntake) intake.stop();
/*
    if (sticks.ButtonR1.pressing()) armMotor.spin(forward);
    else if (sticks.ButtonR2.pressing()) armMotor.spin(reverse);
    else armMotor.stop(); lmfao*/

    clampLatch.check(sticks.ButtonX.pressing());
    clamp.set(clampLatch.state);

    armLatch.check(sticks.ButtonR1.pressing());

    if (armLatch.state == 0) {targetArmPosition = 0;}
    else if (armLatch.state == 1) {targetArmPosition = 124;}
    else if (armLatch.state == 2){targetArmPosition = 666;}
            
    wait(5, msec); 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(skills);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(10, msec);
  }
} 