#include "vex.h"
#include <cmath>
#include "vars.h"
#include "config.h"

using namespace vex;
using namespace std;

//Attempt at position tracking -- we'll see how it goes
int angleChange(){

  //very rudimentary, just degree tracking right now
  while(true){
    curDeg += ((lquad.position(degrees)-prevL) - (rquad.position(degrees)-prevR)) / (lWheelDist + rWheelDist);
    prevL = lquad.position(degrees);
    prevR = rquad.position(degrees);
    task::sleep(20);
  }
  return 1;

}

void track(){

  while (noBitches){

    /*midEncode = bquad.position(degrees);
    leftEncode = lquad.position(degrees);
    rightEncode = rquad.position(degrees);*/

    double leftTrackDis = (lquad.position(degrees) - prevL)/360 * PI * (FourOmni);
    double rightTrackDis = (rquad.position(degrees) - prevR)/360 * PI * (FourOmni);
    double midTrackDis = (bquad.position(degrees) - prevM)/360 * PI * (TwoOmni);

    /*d = (leftTrackDis - rightTrackDis)/14.75; //dist between l/r trackers

    changeDirection = d - direction;*/


    //local offset if moving in straight line
    if (curDeg = 0){

      posx = midTrackDis;
      posy = rightTrackDis;

    }
    //local offset moving on an arc
    else{

      posx = (2 * sin(curDeg/2)) * ((midTrackDis/curDeg) + midTrackDis);
      posy = (2 * sin(curDeg/2)) * ((rightTrackDis/curDeg) + rightTrackDis);

    }

    //avg orientation 𝞱m????
    double avgOrient = heading + (curDeg/2);

    //now we gotta convert to polar, reverse the angle measure, then convert back to rectangular; rotating by -𝞱m
    double polarDeg = sqrt(pow(posx, 2) +pow(posy, 2)) * sin(avgOrient);
    polarDeg = -polarDeg;

  }
}