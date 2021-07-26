#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>


#define TIME_STEP 64
using namespace webots;
using namespace std;

int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Motor *lr;
  lr=robot->getMotor("linear motor");
  Motor *ar;
  ar=robot->getMotor("arm_motor");
  
  
  double linear=0.0; 
  bool flag1 = true;
  
  while (robot->step(TIME_STEP) != -1) {
  
    if(linear > -0.10 && flag1){
        lr->setPosition(linear);
        linear -= 0.004;      
    }else if(linear <= 0){
        flag1 = false;
        lr->setPosition(linear);
        linear += 0.004; 
    }
    
    cout << linear << '\n';

     
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}
