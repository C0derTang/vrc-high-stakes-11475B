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
double driveDist = 0;
double desiredValue;

double posx; // pos = position
double posy;
double heading;

double vectors[3] = {0, 0, 0}; //x, y, and heading

double prevL = 0;
double prevR = 0;
double prevM = 0;
double curDeg = 0;
double targetDeg = 0;

//lateral PID vals
double error = 0;
double prevError = 0;
double derivative;
double totalError = 0; //integral

//turn PID vals
double turnError = 0;
double turnPrevError = 0;
double turnDerivative;
double turnTotalError = 0; //integral
double desiredTurnValue;

// Important settings!
bool enableDrivePID = true;
bool enableTurnPID = true;
bool resetDriveSensors = true;
bool noBitches = true; //conditional for driver control loop -- the code doesn't lie
double turnImportance = 0.5;

//transmission settings
bool transmission = false;
bool tlatch = false;

//constants
const double PI = 3.14159265;
const double lWheelDist = 7.375;
const double rWheelDist = 7.375;
const double mWheelDist = 0.375;
const double TwoOmni = 2.75;
const double FourOmni = 4;

//pneumatic t/f
bool clampOn = false;
bool intakeOn = false;

//ui settings -- fun for victor
bool visual = false;