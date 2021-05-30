//Author : Avishka Sandeepa
//Modified date : 30/5/2021

// Added include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <dos.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

double baseSpeed = 4;
double le = 0;
double set = 3500;
double sensorValues[8];


// All the webots classes are defined in the "webots" namespace
using namespace webots;


//----function to read the values of the sensors and convert to binary-------
void read(){
 for (int i = 0; i < 8; i++){
   if (sensorValues[i] > 5.5){
     sensorValues[i] = 1;
   }else{
     sensorValues[i] = 0;
   }
 }
}
//----------------------end of the read() function----------------------------

//------------------function for PD calculation-------------------------------
double PID_calc(){
 double average = 0;
 double sum = 0;
 for (int i = 0; i < 8 ; i++){ 
   average += sensorValues[i] * i * 1000;
   sum += sensorValues[i];
   
 }
 
 
 double position = average / sum;  //---------weighted mean---------------------
 
 double kp = 0.002;
 double kd = 0.0002;
 //double ki = 0.0;
 double e = position - set;
 double p = kp * e;
 double d = kd * (e - le);
 //double i = ki * (e + le);
 double offset = p + d;
 le = e;
 return offset;
}
//--------------------end of the PID_calc() function---------------------------


//---------------------function for motor driving------------------------------
double Mdriver(double speed){

 if (speed > 0){
   if (speed > 6.28){
     speed = 6.28;
   }
 }else{
   if (speed < -6.28){
     speed = -6.28;
   }
 }
 
 return speed;
}
//---------------------end of the driver() function----------------------------


//---------------------------main function-------------------------------------
int main(int argc, char **argv) {
    Robot *robot = new Robot();
    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    

    //------------------------------------------------------------------------
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
    //------------------------------------------------------------------------

    //--------------junction detecting sensor couple-----------------------

    DistanceSensor *leftMost = robot->getDistanceSensor("leftMost");
    leftMost->enable(TIME_STEP);
    DistanceSensor *rightMost = robot->getDistanceSensor("rightMost");
    rightMost->enable(TIME_STEP);
    //---------------------------------------------------------------------
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    
    //-------------------set position sensors------------------------
    
    PositionSensor *leftPs = robot->getPositionSensor("left_ps");
    leftPs->enable(TIME_STEP);
    PositionSensor *rightPs = robot->getPositionSensor("right_ps");
    rightPs->enable(TIME_STEP);
    
    //----------------------------------------------------------------


    //---------------------main loop-----------------------------------
    while (robot->step(TIME_STEP) != -1){


        // read sensors outputs
        
        for (int i = 0; i < 8 ; i++){
        sensorValues[i] = ir[i]->getValue();
        }
        
        //--------------read junction detection sensor values----------------
        
        double  leftMostValue = leftMost->getValue();
        if (leftMostValue > 5.5){
            leftMostValue = 1;
        }else{
            leftMostValue = 0;
        }
        
        double  rightMostValue = rightMost->getValue();
        if (rightMostValue > 5.5){
            rightMostValue = 1;
        }else{
        rightMostValue = 0;
        }
        
        

        read(); // call a function to get out put as binary values from the IR array
        
        //-------------------read position sensor values------------------------
        double leftPsVal = leftPs->getValue();
        double rightPsVal = rightPs->getValue();
        
        //---------------print the position value as radians-------------------
        std::cout<<"left = "<<leftPsVal<<"  right = "<<rightPsVal<<std::endl;
        std::cout<<"left = "<<leftMostValue<<"  right = "<<rightMostValue<<std::endl;
        //------------------------------testing-------------------------------------
      
        
        //--------------------------------------------------------------------------
        
        double offset = PID_calc(); //get the offset by calling pre defined function
        
        //---------------------set motor speed values to minimize the error------------------------
        
        double left = baseSpeed + offset;
        double right = baseSpeed - offset;
        
        //---call a function to map the above speds within its maximum & minimum speed---
        
        double leftSpeed = Mdriver(left);
        double rightSpeed = Mdriver(right);
        
        
        //----------------------pass the speeds to the motor for run------------------------------
        
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
        
        
        //-------------print the sensor outputs from the IR array & current offset-----------------
        std::cout<<"ir0 = "<<sensorValues[0]<<"  ";
        std::cout<<"ir1 = "<<sensorValues[1]<<"  ";
        std::cout<<"ir2 = "<<sensorValues[2]<<"  ";
        std::cout<<"ir3 = "<<sensorValues[3]<<"  ";
        std::cout<<"ir4 = "<<sensorValues[4]<<"  ";
        std::cout<<"ir5 = "<<sensorValues[5]<<"  ";
        std::cout<<"ir6 = "<<sensorValues[6]<<"  ";
        std::cout<<"ir7 = "<<sensorValues[7]<<std::endl;
            
        std::cout<<" offset : "<<offset<<std::endl;
        
        
        

    }

    delete robot;

    return 0;
}


//
