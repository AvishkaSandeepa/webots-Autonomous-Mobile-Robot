//Author : Avishka Sandeepa
//Modified date : 30/5/2021

// Added include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

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
 
//---------------------main loop---------------------------------------------
 while (robot->step(TIME_STEP) != -1){

//setup motor speeds
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);

 


 

 }

 delete robot;

 return 0;
}


//