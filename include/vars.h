double prevL = 0;
double prevR = 0;
double prevB = 0;
double globalHeading = 0;

double turnImportance = 0.5;

double globalX = 0;
double globalY = 0;

//constants
const double PI = 3.14159265;
const double wheelDiameter = 2.75;
const double leftWheelDist = 4.5;
const double rightWheelDist =4.5;
const double backWheelDist = 1.75;

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

struct PID{
    double kP = 0;
    double kI = 0;
    double kD = 0;

    double cumulativeError = 0;
    double previousError = 0;

    double maxPower = 12.0;

    PID(double p_, double i_, double d_) : kP(p_), kI(i_), kD(d_) {}

    double update(double current, double target){
        double error = target-current;

        cumulativeError += error;
        if(abs(error) < .01 || abs(error) > 20) cumulativeError = 0;

        double derivative = error - previousError;
        previousError = error;

        double power = error*kP + cumulativeError*kI + derivative*kD;
        if (power < -maxPower) power = -maxPower;
        if(power > maxPower) power = maxPower;

        return power;
    }
};


Toggle clampLatch;

PID lateralPID(0,0,0);
PID headingPID(0,0,0);
PID armPID(0,0,0);