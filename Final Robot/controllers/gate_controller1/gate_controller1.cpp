
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 1000

using namespace webots;
using namespace std;

int main (int argc, char** argv) {
    // create the Robot instance.
    Robot* robot = new Robot ();
    Motor* gateMotor = robot->getMotor ("gateMotor");
    gateMotor->setPosition (INFINITY);
    gateMotor->setVelocity (0);

    PositionSensor* gate_ps = robot->getPositionSensor ("gate_ps");
    gate_ps->enable (TIME_STEP);

    int time = 0;

    while ( robot->step (TIME_STEP) != -1 ) {

        if ( time < 1 ) gateMotor->setVelocity (-1.5);
        else if ( 1 <= time && time <= 9 ) gateMotor->setVelocity (0);
        else if ( time < 11 ) gateMotor->setVelocity (1.5);
        else if ( 11 <= time && time < 20 ) gateMotor->setVelocity (0);
        else time = -1;

        time++;


    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
