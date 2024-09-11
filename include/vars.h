//hi

//PID variable tuning (will take fucking forever)
//straight tuning
double kP = 0.05;
double kI = 0.00001;
double kD = 0.02;
//turn
double tkP = 0.05;
double tkI = 0.0001;
double tkD = 0.01;

// Autonomous settings
double latpower = 0;
double driveDist = 0;

double prevL = 0;
double prevR = 0;
double curDeg = 0;
double targetDeg = 0;

//lateral PID vals
double error = 0;
double prevError = 0;
double derivative=0;
double totalError = 0; //integral

//turn PID vals
double turnError = 0;
double turnPrevError = 0;
double turnDerivative=0;
double turnTotalError = 0; //integral

// Important settings!
bool enableDrivePID = true;
bool enableTurnPID = true;
bool noBitches = true; //conditional for driver control loop -- the code doesn't lie
double turnImportance = 0.5;

//transmission settings
bool transmission = false;
bool tlatch = false;

//constants
const double PI = 3.14159265;
const double lWheelDist = 7.375;
const double rWheelDist = 7.375;