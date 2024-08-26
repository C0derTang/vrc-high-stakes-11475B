//hi

//PID variable tuning (will take fucking forever)
//straight tuning
double kP = 0.5;
double kI = 0.2;
double kD = 0.0;
//turn
double tkP = 0.5;
double tkI = 0.2;
double tkD = 0.0;

// Autonomous settings
int desiredValue = 200;
int desiredTurnValue = 0;

//lateral PID vals
int error = 0;
int prevError = 0;
int derivative;
int totalError = 0; //integral

//turn PID vals
int turnError = 0;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError = 0; //integral

//maintains sensor data when driver control starts
bool resetDriveSensors = false;

// Important settings!
bool enableDrivePID = true;
bool noBitches = true; //conditional for driver control loop -- the code doesn't lie
double turnImportance = 0.5;

//transmission settings
bool transmission = false;
bool tlatch = false;