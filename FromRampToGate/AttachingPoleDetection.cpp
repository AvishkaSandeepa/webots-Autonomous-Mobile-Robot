//Author : Avishka Sandeepa
//Modified date : 02/06/2021

// Added include files
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <dos.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

double baseSpeed = 8;
double le = 0;
double set = 3500;
double sensorValues[10];
double mleft;
double mright;

int stage = 1;
int state = 2; // 1==left, 2==right
bool detect = false;
double lpos;
double rpos;
int count = 0;
int c;


// All the webots classes are defined in the "webots" namespace
using namespace webots;


//----function to read the values of the sensors and convert to binary-------
void read(){
 for (int i = 0; i < 8; i++){
   if (sensorValues[i] > 400){
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
 
 double kp = 0.0006;
 double kd = 0.0003;
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
   if (speed > 10){
     speed = 10;
   }
 }else{
   if (speed < -10){
     speed = -10;
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
    DistanceSensor *ir[10];
    char sensorNames[10][5] = {
    "ir0", "ir1", "ir2", "ir3", "ir4",
    "ir5", "ir6", "ir7", "lm", "rm"
    };
    
    //enable the sensors to get measurements
    for (int i = 0; i < 10; i++) {
    ir[i] = robot->getDistanceSensor(sensorNames[i]);
    ir[i]->enable(TIME_STEP);
    }
    //------------------------------------------------------------------------

    //--------------junction detecting sensors-----------------------

    //DistanceSensor *leftMost = robot->getDistanceSensor("lm");
    //leftMost->enable(TIME_STEP);
    //DistanceSensor *rightMost = robot->getDistanceSensor("rm");
    //rightMost->enable(TIME_STEP);
    //DistanceSensor *left = robot->getDistanceSensor("left");
    //left->enable(TIME_STEP);
    //DistanceSensor *mid = robot->getDistanceSensor("mid");
    //mid->enable(TIME_STEP);
    //DistanceSensor *right = robot->getDistanceSensor("right");
    //right->enable(TIME_STEP);
    //DistanceSensor *rightM = robot->getDistanceSensor("rightM");
    //rightM->enable(TIME_STEP);
    //DistanceSensor *leftM = robot->getDistanceSensor("leftM");
    //leftM->enable(TIME_STEP);
    //---------------------------------------------------------------------
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    
    //-------------------set position sensors------------------------
    
    PositionSensor *leftPs = robot->getPositionSensor("left wheel sensor");
    leftPs->enable(TIME_STEP);
    PositionSensor *rightPs = robot->getPositionSensor("right wheel sensor");
    rightPs->enable(TIME_STEP);
    
    //----------------------------------------------------------------
    
    
    //---------------------main loop-----------------------------------
    while (robot->step(TIME_STEP) != -1){


        // read sensors outputs
        
        for (int i = 0; i < 10 ; i++){
        sensorValues[i] = ir[i]->getValue();
        }
        
        
        //--------------read junction detection sensor values----------------
        read(); // call a function to get out put as binary values from the IR array
        double  leftMostValue = sensorValues[8];
        double  rightMostValue = sensorValues[9];
        
        if (leftMostValue > 800){
            leftMostValue = 1;
        }else{
            leftMostValue = 0;
        }
        
        //double  rightMostValue = rightMost->getValue();
        if (rightMostValue > 800){
            rightMostValue = 1;
        }else{
        rightMostValue = 0;
        }
        
        
        /*
        double  leftValue = left->getValue();
        if (leftValue > 5.5){
            leftValue = 1;
        }else{
        leftValue = 0;
        }
        
        double  rightValue = right->getValue();
        if (rightValue > 5.5){
            rightValue = 1;
        }else{
        rightValue = 0;
        }
        
        double  midValue = mid->getValue();
        if (midValue > 5.5){
            midValue = 1;
        }else{
        midValue = 0;
        }
        
        
        double  leftMValue = leftM->getValue();
        if (leftMValue > 5.5){
            leftMValue = 1;
        }else{
        leftMValue = 0;
        }
        
        double  rightMValue = rightM->getValue();
        if (rightMValue > 5.5){
            rightMValue = 1;
        }else{
        rightMValue = 0;
        }
        */
        

        
        
        //-------------------read position sensor values------------------------
        double leftPsVal = leftPs->getValue();
        double rightPsVal = rightPs->getValue();
        
        //---------------print the position value as radians-------------------
        std::cout<<"left = "<<leftPsVal<<"  right = "<<rightPsVal<<std::endl;
        std::cout<<"leftmost = "<<leftMostValue<<"  rightmost = "<<rightMostValue<<std::endl;
        //------------------------------testing-------------------------------------
        
            
        
        //if (count > 50){
        
          if (stage==1){
          
            
            if (leftMostValue==1 && rightMostValue==0 ){
              stage = 2;
              detect = true;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###left detected#### "<<count<<std::endl;
              count = 0;
            }else if (rightMostValue==1 && leftMostValue==0 ){
              stage = 3;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"####right detected#### "<<count<<std::endl;
              count = 0;
            
            }else if (leftMostValue==1 && rightMostValue==1 ){
              stage = 4;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"####hey fucker#### "<<count<<std::endl;
              count = 0;
            
            }else if(sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
              stage = 5;
              
            
            }else{
            
            count = 0;
            
            double offset = PID_calc(); //get the offset by calling pre defined function
            
            //---------------------set motor speed values to minimize the error------------------------
            
            double left = baseSpeed + offset;
            double right = baseSpeed - offset;
            
            //---call a function to map the above speds within its maximum & minimum speed---
            
            double leftSpeed = Mdriver(left);
            double rightSpeed = Mdriver(right);
            
            
            mleft = leftSpeed;
            mright = rightSpeed;
            
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
          
          }else if (stage == 2){
            if ((leftPsVal < lpos + 3.8) || (rightPsVal < rpos + 3.8)){
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(8);
              
              mleft = 8;
              mright = 8;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[3]==0){
                std::cout<<"*******stopped then turn left*****"<<std::endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(8);
                
                mleft = 0;
                mright = 8;
              }else{
                //count = 0;
                stage = 1;
              }
                
              
            }
          
          
          }else if (stage == 3){
            if ((leftPsVal < lpos + 3.8) || (rightPsVal < rpos + 3.8)){
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(8);
              
              mleft = 8;
              mright = 8;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[4]==0){
                std::cout<<"*******stopped then turn right*****"<<std::endl;
                leftMotor->setVelocity(8);
                rightMotor->setVelocity(0);
                
                mleft = 8;
                mright = 0;
              }else{
                //count = 0;
                stage = 1;
              }
                
              
            }
          
          }else if (stage == 4 && state == 1){
            if ((leftPsVal < lpos + 3.8) || (rightPsVal < rpos + 3.8)){
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(8);
              
              mleft = 8;
              mright = 8;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[4]==0){
                std::cout<<"*******stopped then turn left*****"<<std::endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(8);
                
                mleft = 0;
                mright = 8;
              }else{
                //count = 0;
                stage = 6;
              }
                
              
            }
          
          
          }else if (stage == 4 && state == 2){
            if ((leftPsVal < lpos + 3.8) || (rightPsVal < rpos + 3.8)){
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(8);
              
              mleft = 8;
              mright = 8;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[4]==0){
                std::cout<<"*******stopped then turn right*****"<<std::endl;
                leftMotor->setVelocity(8);
                rightMotor->setVelocity(0);
                
                mleft = 8;
                mright = 0;
              }else{
                //count = 0;
                stage = 7;
              }
                
              
            }
          
          }else if (stage == 5){
            count++;
            if (count < 7){
              leftMotor->setVelocity(mleft);
              rightMotor->setVelocity(mright);
              stage = 1;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              stage = 1;
              break;
              //count = 0;
            
            }
            
          
          }else if (stage==6){
            noOfPoles=0
            double baseSpeed = 4;
            while (leftMostValue==0 && rightMostValue==0){
            count = 0;
            
            double offset = PID_calc(); //get the offset by calling pre defined function
            
            //---------------------set motor speed values to minimize the error------------------------
            
            double left = baseSpeed + offset;
            double right = baseSpeed - offset;
            
            //---call a function to map the above speds within its maximum & minimum speed---
            
            double leftSpeed = Mdriver(left);
            double rightSpeed = Mdriver(right);
            
            
            mleft = leftSpeed;
            mright = rightSpeed;
            
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
                
            std::cout<<" offset : "<<offset<<std::endl;}

            if (leftMostValue==1 && rightMostValue==0 && noOfPoles==state){
              stage = 2;
              detect = true;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###left detected#### "<<count<<std::endl;
              count = 0;}
            else if (leftMostValue==1 && rightMostValue==0 && noOfPoles!=state){
              stage=5
            }
          }

          else if (stage==7){
            noOfPoles=0
            double baseSpeed = 4;
            while (leftMostValue==0 && rightMostValue==0){
            count = 0;
            
            double offset = PID_calc(); //get the offset by calling pre defined function
            
            //---------------------set motor speed values to minimize the error------------------------
            
            double left = baseSpeed + offset;
            double right = baseSpeed - offset;
            
            //---call a function to map the above speds within its maximum & minimum speed---
            
            double leftSpeed = Mdriver(left);
            double rightSpeed = Mdriver(right);
            
            
            mleft = leftSpeed;
            mright = rightSpeed;
            
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
                
            std::cout<<" offset : "<<offset<<std::endl;}

            if (leftMostValue==0 && rightMostValue==1 && noOfPoles==state){
              stage = 3;
              detect = true;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###right detected#### "<<count<<std::endl;
              count = 0;}
            else if (leftMostValue==1 && rightMostValue==0 && noOfPoles!=state){
              stage=5
            }
          }

          
          
          
        
        
        //-------------------------------
        //}else{
        //  leftMotor->setVelocity(3);
        //  rightMotor->setVelocity(3);
        //}
       
        //--------------------------------------------------------------------------
        /*
        read(); // call a function to get out put as binary values from the IR array
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
            
        std::cout<<" offset : "<<offset<<std::endl; */
        
        
    std::cout<<"  count =  "<<count<<std::endl;
    std::cout<<"        "<<std::endl;
    }

    delete robot;

    return 0;
}


//left wheel sensor right wheel sensor
