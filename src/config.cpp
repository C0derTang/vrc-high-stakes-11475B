//robot configuration file
#include "vex.h"

using namespace vex;

// define your global instances of motors and other devices here
brain Thinky;

controller sticks;

// Driving/Transmission motors
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

//Intake
motor intake(3, false);

//claw
motor claw(4, false);

//shaft encoders
encoder lquad = encoder(Thinky.ThreeWirePort.A);
encoder rquad = encoder(Thinky.ThreeWirePort.C);
encoder bquad = encoder(Thinky.ThreeWirePort.E);

digital_out clamp = digital_out(Thinky.ThreeWirePort.G);
digital_out intakeCyl = digital_out(Thinky.ThreeWirePort.H);
