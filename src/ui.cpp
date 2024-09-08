//ui on brain while running -- I wanna have a toggle between variable values and a vector on a graph

#include "vars.h"
#include "vex.h"

using namespace std;
using namespace vex;

// x, y, heading, program name, object detected, systems check
int uidata(void) {

    while (!visual) {
        text();

        task::sleep(20);
    }

    return 1;
}

void text(void) {


}

void graph(void) {

}