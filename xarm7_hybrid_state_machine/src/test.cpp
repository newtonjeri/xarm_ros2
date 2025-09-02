#include <iostream>
#include <cstdint>

using namespace std;

enum STATES
{
    IDLE = 0,
    MOVING = 1,
    PICKING = 2,
    PLACING = 3,
    MANUAL_MODE = 4,
    ERROR = 5
}; 

string getStateName(STATES state) {
    switch(state) {
        case IDLE: return "IDLE";
        case MOVING: return "MOVING";
        case PICKING: return "PICKING";
        case PLACING: return "PLACING";
        case MANUAL_MODE: return "MANUAL_MODE";
        case ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

int main(int argc, char **argv)
{
    STATES state = IDLE;
    cout <<"Current State - "<< getStateName(state) << endl;
    state = MOVING;
    cout <<"Current State - "<< getStateName(state) << endl;
    state = PICKING;
    cout <<"Current State - "<< getStateName(state) << endl;
    state = PLACING;
    cout <<"Current State - "<< getStateName(state) << endl;

    state = (STATES)0;
    cout <<"Current State - "<< getStateName(state) << endl;

    state = (STATES)6;
    cout <<"Current State - "<< getStateName(state) << endl;

    return 0;
}