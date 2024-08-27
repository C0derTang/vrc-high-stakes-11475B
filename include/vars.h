//hi

//universal value for pi -- truncated to fifth decimal for consistency
double apple = 3.14159;

//wheel sizes inches -- we'll probaby forget soooo
double FourOmni = 4.125;
double TwoOmni = 2.75;

//tracking wheel vars -- will be fun!!
//starting with defining robot's starting vectors
//think a reliable reference for direction is positive side at 0 degrees
double posy = 0;
double posx = 0;
double direction = 0;

//perpendicular distance from center of rotation to tracking wheels -- will update later
double rightTrackDist = 0;
double leftTackDist = 0;
double midTrackDist = 0;

//current encoder values
int leftEncode;
int rightEncode;
int midEncode;

//dist traveled per wheel; f/- b/+  l/- r/+
double leftDisplace;
double rightDisplace;
double midDisplace;

//array of doubles noting current tracking information from the tracking algorithm, {x,  y, 𝞱}
double trackingInfo[3] = {0, 0, 0};

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