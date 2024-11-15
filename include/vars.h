//hi

//PID variable tuning (will take fucking forever)
//straight tuning
double kP = 0.03;
double kI = 0.00001;
double kD = 0.05;
//turn
double tkP = .15;
double tkI = 0.000005;
double tkD =0.05;

// Autonomous settings
double lpower = 0;
double rpower = 0;
double driveDist = 0;

double prevL = 0;
double prevR = 0;
double prevS = 0;
double curDeg = 0;

double turnImportance = 0.5;

double xPos = 0;
double yPos = 0;

//constants
const double PI = 3.14159265;
const double wheelDiameter = 4.125;
const double lWheelDist = 7.375;
const double rWheelDist = 7.375;
const double sWheelDist = 7.375;