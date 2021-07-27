//Author : Avishka Sandeepa
//Modified date : 01/06/2021

// Added include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;
#define TIME_STEP 64
#define MAX_SPEED 6.28

double stage = 1;

//---------------------------main function-------------------------------------
int main(int argc, char **argv) {
    Robot *robot = new Robot();
    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor *lm1 = robot->getMotor("lm1");
    Motor *lm2 = robot->getMotor("lm2");
    
    Motor *rm1 = robot->getMotor("rm1");
    Motor *rm2 = robot->getMotor("rm2");
    
    Motor *rmidm = robot->getMotor("rmidm");
    Motor *lmidm = robot->getMotor("lmidm");
    
    Motor *rotate90degreeM1 = robot->getMotor("rotate90degreeM1");
    Motor *rotate90degreeM2 = robot->getMotor("rotate90degreeM2");
    
    lm1->setPosition(INFINITY);
    lm2->setPosition(INFINITY);
    rm1->setPosition(INFINITY);
    rm2->setPosition(INFINITY);
    rmidm->setPosition(INFINITY);
    lmidm->setPosition(INFINITY);
    
    rotate90degreeM1->setPosition(INFINITY);
    rotate90degreeM2->setPosition(INFINITY);

    lm1->setVelocity(0);
    lm2->setVelocity(0);
    rm1->setVelocity(0);
    rm2->setVelocity(0);
    rmidm->setVelocity(0);
    lmidm->setVelocity(0);
    
    rotate90degreeM1->setVelocity(0);
    rotate90degreeM2->setVelocity(0);
    
    //-------------------set position sensors------------------------
    
    PositionSensor *lp1 = robot->getPositionSensor("lp1");
    lp1->enable(TIME_STEP);
    PositionSensor *lp2 = robot->getPositionSensor("lp2");
    lp2->enable(TIME_STEP);
    
    PositionSensor *rp1 = robot->getPositionSensor("rp1");
    rp1->enable(TIME_STEP);
    PositionSensor *rp2 = robot->getPositionSensor("rp2");
    rp2->enable(TIME_STEP);
    
    
    PositionSensor *rmidp = robot->getPositionSensor("rmidp");
    rmidp->enable(TIME_STEP);
    PositionSensor *lmidp = robot->getPositionSensor("lmidp");
    lmidp->enable(TIME_STEP);
    
    PositionSensor *rotate90degreePS1 = robot->getPositionSensor("rotate90degreePS1");
    rotate90degreePS1->enable(TIME_STEP);
    PositionSensor *rotate90degreePS2 = robot->getPositionSensor("rotate90degreePS2");
    rotate90degreePS2->enable(TIME_STEP);
    
    //----------------------------------------------------------------
    
    
    //---------------------main loop-----------------------------------
    while (robot->step(TIME_STEP) != -1){
    
    double lp1_Val = lp1->getValue();
    double lp2_Val = lp2->getValue();
    double rp1_Val = rp1->getValue();
    double rp2_Val = rp2->getValue();
    double rmidp_Val = rmidp->getValue();
    double lmidp_Val = lmidp->getValue();
    double rotate90degreePS1_val = rotate90degreePS1->getValue();
    double rotate90degreePS2_val = rotate90degreePS2->getValue();
    
    //std::cout<<"lp_1"<<lp1_Val<<std::endl;
    std::cout<<"lp 1 = "<<lp1_Val<<" lp 2 = "<<lp2_Val<<" rp 1 = "<<rp1_Val<<" rp 2 = "<<rp2_Val<<" left mid = "<<lmidp_Val<<" right mid = "<<rmidp_Val<<std::endl;
    
    if (stage == 1){
      if  (lp1_Val > 0.15 || rp1_Val > 0.15){
        lm1->setVelocity(-2);
        rm1->setVelocity(-2);
    }else{
      lm1->setVelocity(0);
      rm1->setVelocity(0);
      stage = 2;
      }
      
    }else if (stage == 2){
      if(lp2_Val > -0.7 || rp2_Val < 0.7){
        lm2->setVelocity(-1);
        rm2->setVelocity(1);
      }else{
        lm2->setVelocity(0);
        rm2->setVelocity(0);
        stage = 3;
      }
      
    }else if(stage == 3){
      if  (lp1_Val < 1.5 || rp1_Val < 1.5){
        lm1->setVelocity(1);
        rm1->setVelocity(1);
    }else{
      lm1->setVelocity(0);
      rm1->setVelocity(0);
      stage =4;
    }
    
    // rotation happenss here    
    }else if(stage == 4){
      if  (rotate90degreePS1_val > -1 || rotate90degreePS2_val > -1){
        rotate90degreeM1->setVelocity(-1);
        rotate90degreeM2->setVelocity(-1);
    }else{
      rotate90degreeM1->setVelocity(0);
      rotate90degreeM2->setVelocity(0);
      stage = 5;
      
    } 
    // ================================    
    }else if(stage == 5){
      if(lp1_Val > 0.15 || rp1_Val > 0.15){
        lm1->setVelocity(-2);
        rm1->setVelocity(-2);
    }else{
      lm1->setVelocity(0);
      rm1->setVelocity(0);
      stage = 6;
      }
      
    }else if (stage == 6){
      if(lp2_Val < -0.064 || rp2_Val > 0.064){
        lm2->setVelocity(1);
        rm2->setVelocity(-1);
      }else{
        lm2->setVelocity(0);
        rm2->setVelocity(0);
        break;
      }
      
    }
    
   }

    delete robot;

    return 0;
}


//left wheel sensor right wheel sensor
