
// Authors        : Team BRAND
// Modified date  : 23/07/2021
// License        : MIT

//==============================================================================
//*                                  Preamble                                  *
//==============================================================================

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

double baseSpeed = 5;
double le = 0;
double set = 3500;
double sensorValues[10];
double lpos;
double rpos;
double turn = 8;
int state =2;
int count =0; // for dotted line

// Variables related to take turns and wheels
float advancedBy = 0.9;        // distance of free move when a junction is detected.
float forward_speed = 5;    // free moving speed
float sharpturn_speed = 5;  // speed of taking turns
double mleft;
double mright;

// Variables related to state of the task
int stage = 1;
bool detect = false;
double pos;
int c;

//==============================================================================
//*                         Defining custom functions                          *
//==============================================================================

//---------Reading the values of the sensors and convert to binary 1/0----------
void read(){
  for (int i = 0; i < 10; i++){
    if (sensorValues[i] < 800){
      sensorValues[i] = 1;
    }else{
      sensorValues[i] = 0;
    }
  }
}
//--------------------------end of the read() function--------------------------

//-------------------------Function for PD calculation--------------------------
double PID_calc(){
  double average = 0;
  double sum = 0;
  for (int i = 0; i < 8 ; i++){
    average += sensorValues[i] * i * 1000;
    sum += sensorValues[i];
  }

  double position = average / sum; // current position

  double kp = 0.008;
  double kd = 0.0004;
  double e = position - set; // deviation from the set position
  double p = kp * e;
  double d = kd * (e - le);
  double offset = p + d;
  le = e;
  return offset;
}
//------------------------end of the PID_calc() function------------------------

//--------------------------Function for motor driving--------------------------
double Mdriver(double speed){

  if (speed > 0){
    if (speed > MAX_SPEED){
      speed = MAX_SPEED;
    }
  }else{
    if (speed < -MAX_SPEED){
      speed = -MAX_SPEED;
    }
  }
  return speed;
}
//---------------------end of the driver() function----------------------------


//==============================================================================
//*                                                                            *
//*                        Main function for the robot                         *
//*                                                                            *
//==============================================================================

int main(int argc, char **argv) {

  Robot *robot = new Robot(); // initializing the robot object

  // get a handler to the motors and set target position to infinity
  Motor *leftMotor  = robot->getMotor("left motor");
  Motor *rightMotor = robot->getMotor("right motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  // set the initial velocities
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);

  //set position sensors
  PositionSensor *leftPs = robot->getPositionSensor("left_ps");
  leftPs->enable(TIME_STEP);
  PositionSensor *rightPs = robot->getPositionSensor("right_ps");
  rightPs->enable(TIME_STEP);

  //----------------------------------------------------------------------------

  // initialize infrared sensors
  char sensorNames[10][10] = {
    "ir0", "ir1", "ir2", "ir3", "ir4","ir5", "ir6", "ir7", // for PID_calc()
    "leftmost", "rightmost" // for junction detection
  };

  // enable the sensors to get measurements
  DistanceSensor *ir[10];
  for (int i = 0; i < 10; i++) {
    ir[i] = robot->getDistanceSensor(sensorNames[i]);
    ir[i]->enable(TIME_STEP);
  }
  //----------------------------------------------------------------------------

  //============================================================================
  //*                                 main loop                                *
  //============================================================================

  while (robot->step(TIME_STEP) != -1){

    // read sensors outputs
    for (int i = 0; i < 10 ; i++){
      sensorValues[i] = ir[i]->getValue();
    }

    //read junction detection sensor values
    read(); // call a function to get out put as binary values from the IR array
    double  leftMostValue = sensorValues[8];
    double  rightMostValue = sensorValues[9];

    //read position sensor values-
    double leftPsVal = leftPs->getValue();
    double rightPsVal = rightPs->getValue();

    //print the position value as radians
    cout<< "Left PS = " << leftPsVal;
    cout<<"  Right PS = " <<rightPsVal<<'\n';
    cout<< "Left Most IR = " <<leftMostValue;
    cout<<"  Right Most IR = " <<rightMostValue<<'\n';

    //------------------------------testing-------------------------------------

    if (stage==1){

      if (leftMostValue==1 && rightMostValue==0 ){
        stage = 2;
        //detect = true;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"=========left detected========= "<<count<<'\n';
        count = 0;
      }else if (rightMostValue==1 && leftMostValue==0 ){
        stage = 3;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"right detected"<<count<<'\n';
        count = 0;
      }else if (leftMostValue==1 && rightMostValue==1 ){
        stage = 4;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"#### T junction detected #### "<<count<<'\n';
        count = 0;
      }else if(sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        stage = 5;
      }else{
        count = 0;
        double offset = PID_calc(); //get the offset by calling pre defined function

        //set motor speed values to minimize the error
        double left = baseSpeed + offset;
        double right = baseSpeed - offset;

        //Call a function to map the above speds within its maximum & minimum speed
        double leftSpeed = Mdriver(left);
        double rightSpeed = Mdriver(right);
        mleft = leftSpeed;
        mright = rightSpeed;
        //pass the speeds to the motor for run
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        //print the sensor outputs from the IR array & current offset
        cout<<"ir0 = "<<sensorValues[0]<<"  ";
        cout<<"ir1 = "<<sensorValues[1]<<"  ";
        cout<<"ir2 = "<<sensorValues[2]<<"  ";
        cout<<"ir3 = "<<sensorValues[3]<<"  ";
        cout<<"ir4 = "<<sensorValues[4]<<"  ";
        cout<<"ir5 = "<<sensorValues[5]<<"  ";
        cout<<"ir6 = "<<sensorValues[6]<<"  ";
        cout<<"ir7 = "<<sensorValues[7]<<'\n';
        cout<<" offset : "<<offset<<'\n';
      }

    }else if (stage == 2){
      if ((leftPsVal < lpos + advancedBy) || (rightPsVal < rpos + advancedBy)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);

        //creating a memory to save wheels current speeds
        mleft = forward_speed; mright = forward_speed;
      }else{
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        mleft = 0; mright = 0;
        // advancing is over.
        // taking the left turn
        if(rightPsVal < rpos + advancedBy + turn){
          cout<<"*******stopped then turn left*****"<<'\n';
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(sharpturn_speed);

          mleft = 0; mright = sharpturn_speed;
        }else{
          //count = 0;
          stage = 1;
        }
      }


    }else if (stage == 3){
      if ((leftPsVal < lpos + advancedBy) || (rightPsVal < rpos + advancedBy)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);

        //creating a memory to save wheels current speeds
        mleft = forward_speed;  mright = forward_speed;
      }else{
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        mleft = 0; mright = 0;
        // advancing is over.
        // takjing the left turn
        if(leftPsVal < lpos + advancedBy + turn){
          cout<<"*******stopped then turn right*****"<<'\n';
          leftMotor->setVelocity(sharpturn_speed);
          rightMotor->setVelocity(0);
          mleft = sharpturn_speed; mright = 0;
        }else{
          stage = 1;
        }
      }

    }else if (stage == 4 && state == 1){
      if ((leftPsVal < lpos + advancedBy) || (rightPsVal < rpos + advancedBy)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);

        //creating a memory to save wheels current speeds
        mleft = forward_speed;  mright = forward_speed;

      }else{

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        mleft = 0; mright = 0;

        if(rightPsVal < rpos + advancedBy + turn){
          cout<<"*******stopped then turn left*****"<<'\n';
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(sharpturn_speed);

          mleft = 0; mright = sharpturn_speed;

        }else{
          //count = 0;
          stage = 1;
        }


      }


    }else if (stage == 4 && state == 2){
      if ((leftPsVal < lpos + advancedBy) || (rightPsVal < rpos + advancedBy)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);
        //creating a memory to save wheels current speeds
        mleft = forward_speed;  mright = forward_speed;
      }else{
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        mleft = 0; mright = 0;

        if(leftPsVal < lpos + advancedBy + turn){
          cout<<"*******stopped then turn right*****"<<'\n';
          leftMotor->setVelocity(sharpturn_speed);
          rightMotor->setVelocity(0);

          mleft = sharpturn_speed; mright = 0;
        }else{
          //count = 0;
          stage = 1;
        }
      }
      //-----------------------------dotted line area-----------------------------
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
        
      }
    }
    cout <<"  count =  "<<count<<'\n';
    cout <<"        "<<'\n';
    count ++;
  } // end of main while loop

  delete robot;
  return 0;
}
