//hi

//PID variable tuning (will take fucking forever)
//straight tuning
double kP = 0.05;
double kI = 0.00000001;
double kD = 0.02;
//turn
double tkP = .3;
double tkI = 0.00000005;
double tkD = 0.01;

// Autonomous settings
double lpower = 0;
double rpower = 0;
double driveDist = 0;

double prevL = 0;
double prevR = 0;
double curDeg = 0;
double targetDeg = 0;

//lateral PID vals for left and right
double lerror = 0;
double lprevError = 0;
double lderivative=0;
double ltotalError = 0; //integral
double rerror = 0;
double rprevError = 0;
double rderivative=0;
double rtotalError = 0; //integral

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

bool clamped = false;

//constants
const double PI = 3.14159265;
const double lWheelDist = 7.375;
const double rWheelDist = 7.375;

// universal object for toggle states, makes variable management easier
struct Toggle{
    bool state = false;
    bool latch = false;

    void check(bool cond){
      if (cond){
        if (!latch){
          state  = !state;
          latch = true;
        }
        }else{
        latch = false;
        }
    }
};

Toggle tlatch;
  Toggle clatch;
  Toggle a1latch;
  Toggle a2latch;
