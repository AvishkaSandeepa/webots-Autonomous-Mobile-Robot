//Adding required libraries
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <dos.h>
using namespace webots;
using namespace std;

// defining variables
#define TIME_STEP 64
#define MAX_SPEED 10


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  Motor *leftMotor  = robot->getMotor("left motor");
  Motor *rightMotor = robot->getMotor("right motor");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  // set the initial velocities
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);

  //set position sensors
    DistanceSensor*right_ultrasound = robot->getDistanceSensor("right_ultrasound");
    DistanceSensor*left_ultrasound = robot->getDistanceSensor("left_ultrasound");

    right_ultrasound->enable(TIME_STEP);
    left_ultrasound->enable(TIME_STEP);

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    leftMotor->setVelocity(1);
    rightMotor->setVelocity(1);
    // Process sensor data here.
    
    
    double R_DS=right_ultrasound->getValue();
    double L_DS=left_ultrasound->getValue();
    
     cout << "right = " <<R_DS << "  left = " << L_DS <<'\n';
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
