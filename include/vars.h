//constants
const double PI = 3.14159265;
const double wheelDiameter = 2.75;
const double leftWheelDist = 4.5;
const double rightWheelDist =4.5;
const double backWheelDist = 1.75;

double prevL = 0;
double prevR = 0;
double prevB = 0;

double turnImportance = 0.3;

double globalX = 0;
double globalY = 0;
double globalHeading = 0;

double targetArmPosition = 0;

struct Toggle{
    int states = 2;
    int state = 0;
    bool latch = false;

    Toggle(int s_) : states(s_) {}

    void check(bool cond){
      if (cond){
        if (!latch){
          state = (state+1)%states;
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

    PID(double p_, double i_, double d_) : kP(p_), kI(i_), kD(d_) {}

    double update(double current, double target, double maxPower){
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