//robot configuration file
#include "vex.h"

using namespace vex;

extern brain Thinky;

extern controller sticks;

// Driving/Transmission motors
extern motor lm1;
extern motor lm2;
extern motor lm3;
extern motor rm1;
extern motor rm2;
extern motor rm3;

extern motor_group leftMotor1;
extern motor_group rightMotor1;

extern motor_group leftMotor;
extern motor_group rightMotor;

//Intake
extern motor intake;

//claw
extern motor claw;

//sensors
vision sony;
distance ruler;
inertial inert;

//shaft encoders
extern encoder lquad;
extern encoder rquad;
extern encoder bquad;

extern digital_out clamp;
extern digital_out intakeCyl;