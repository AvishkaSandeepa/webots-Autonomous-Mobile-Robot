//Author : Avishka Sandeepa
//Modified date : 30/5/2021

// Added include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

double sensorValues[8];

// All the webots classes are defined in the "webots" namespace
using namespace webots;


//---------------------main function----------------------------

int main(int argc, char **argv) {
    Robot *robot = new Robot();
 
    // get a handler to the motors and set target position to infinity (speed control)
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    //initialize the sensors for line detection-------------

    // initialize sensors
    DistanceSensor *ir[8];
    char sensorNames[8][4] = {
    "ir0", "ir1", "ir2", "ir3",
    "ir4", "ir5", "ir6", "ir7"
    };
    


    //enable the sensors to get measurements
    for (int i = 0; i < 8; i++) {
        ir[i] = robot->getDistanceSensor(sensorNames[i]);
        ir[i]->enable(TIME_STEP);
    }
 
//---------------------main loop---------------------------------------------
    while (robot->step(TIME_STEP) != -1){

    //------------get the values from the sensors------------------
        for (int i = 0; i < 8 ; i++){
            sensorValues[i] = ir[i]->getValue();
        }
    //setup motor speeds
        leftMotor->setVelocity(MAX_SPEED);
        rightMotor->setVelocity(MAX_SPEED);

    


    

    }

    delete robot;

    return 0;
}


//