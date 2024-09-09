//ui on brain while running -- I wanna have a toggle between variable values and a vector on a graph

#include "vars.h"
#include "vex.h"
#include "config.h"
#include <string>
#include <sstream>
#include <iostream>

using namespace std;
using namespace vex;

ostringstream px;
ostringstream py;
ostringstream h;

string tf;

// x, y, heading, program name, systems check
int uidata(void) {

    if (systemsCheck()){
        tf = "True";
    }
    else{
        tf = "False";
    }

    while(noBitches){
        while (!visual) {
            text();
            task::sleep(20);
        }

        graph();
        task::sleep(20);

        }
    return 1;
}


//text form
void text(void) {

    Thinky.Screen.clearScreen();

    px << posx;
    py << posy;
    h << heading;

    Thinky.Screen.print("x: " + px.str() + " | y: " + py.str() + " | Facing: " + h.str() + 

                        "\nProg: " + programName + " | System Check: " + tf);

}


//graph form
void graph(void) {



}

//systems check
bool systemsCheck(void){



    return false;
}