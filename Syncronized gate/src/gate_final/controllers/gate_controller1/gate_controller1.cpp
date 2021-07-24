
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 64

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();
    Motor *gateMotor  = robot->getMotor("gateMotor");
    gateMotor->setPosition(INFINITY);
    gateMotor->setVelocity(0);
    PositionSensor *gate_ps = robot->getPositionSensor("gate_ps");
    
    gate_ps->enable(TIME_STEP);
    
    while (robot->step(TIME_STEP) != -1) {
      
      double PSVal = gate_ps->getValue();
      gateMotor->setVelocity(-2);
      
      
      
      
      cout << "Position = " << PSVal << '\n';
      
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
