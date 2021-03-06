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

double baseSpeed = 4;
double le = 0;
double set = 3500;
double sensorValues[10];
double lpos;
double rpos;
double turn = 8.5;
int state =3; // after the circular area
int count =0; // for dotted line

//variables for wall following
float kpp = 0.5;//0.005;
float kdd = 0.1;//0
double right_prev_wall = 0;
double left_prev_wall = 0;
double mid_prev_wall = 0;
double distance_to_wall = 16;


// Variables related to take turns and wheels
float advancedBy = 0.7; //0.9;       // distance_to_wall of free move when a junction is detected.
float forward_speed = 4;    // free moving speed
float sharpturn_speed = 4;  // speed of taking turns
double mleft;
double mright;

// Variables related to state of the task
int stage = 1;
bool detect = false;
double pos;
int c;
int countB=0;
int detectingTurn=1;
//Variables for pillar detecting
float advancedonRamp = 1.5;
int noOfPoles=0;
bool flagPillar=false;
int colordif=2;
int wrongPillar=0;
int finishedcircle=1;

//==============================================================================
//*                         Defining custom functions                          *
//==============================================================================

//---------Reading the values of the sensors and convert to binary 1/0----------
void read(){
  for (int i = 0; i < 10; i++){
    if (sensorValues[i] < 700){
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
  double kd = 0.0002;
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

  // initializing distance_to_wall sensors
  DistanceSensor*right_ultrasound = robot->getDistanceSensor("right_ultrasound");
  DistanceSensor*left_ultrasound = robot->getDistanceSensor("left_ultrasound");

  right_ultrasound->enable(TIME_STEP);
  left_ultrasound->enable(TIME_STEP);

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

    double R_DS=right_ultrasound->getValue();
    double L_DS=left_ultrasound->getValue();



    //print the position value as radians
    cout<< "Left PS = " << leftPsVal;
    cout<<"  Right PS = " <<rightPsVal<<'\n';
    cout<< "Left Most IR = " <<leftMostValue;
    cout<<"  Right Most IR = " <<rightMostValue<<'\n';


    //------------------------------testing-------------------------------------

    if (stage==1){

      if (leftMostValue==1 && rightMostValue==0 ){
        if (countB>detectingTurn){
        countB=0;
        stage = 2;
        //detect = true;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"=========left detected========= "<<count<<'\n';
        count = 0;}
        else{
          leftMotor->setVelocity(2);
          rightMotor->setVelocity(2);
          countB=countB+1;
          cout<<"=========CountB========= "<<countB<<'\n';
        }
      }else if (rightMostValue==1 && leftMostValue==0 ){
        if (countB>detectingTurn){
        countB=0;
        stage = 3;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"right detected"<<count<<'\n';
        count = 0;}
        else{
          leftMotor->setVelocity(2);
          rightMotor->setVelocity(2);
          countB=countB+1;
          cout<<"=========CountB========= "<<countB<<'\n';
        }
      }else if (leftMostValue==1 && rightMostValue==1 ){
        if (countB>detectingTurn){
        countB=0;
        stage = 4;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"############################################################ T junction detected ######################################################################## "<<count<<'\n';
        count = 0;}
        else{
          leftMotor->setVelocity(2);
          rightMotor->setVelocity(2);
          countB=countB+1;
          cout<<"=========CountB========= "<<countB<<'\n';
        }
      }else if(sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        stage = 5;
      }else if((R_DS <=15 || L_DS <=15)){
        stage = 20;
        cout << "Wall following started" << '\n';
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
        if (wrongPillar==1){
            leftMotor->setVelocity(15);
            rightMotor->setVelocity(15);
            wrongPillar=0;
            stage=6;
        }
        // taking the left turn
        else {
        // taking the left turn
        if(rightPsVal < rpos + advancedBy + turn){
          cout<<"*******stopped then turn left*****"<<'\n';
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(sharpturn_speed);

          mleft = 0; mright = sharpturn_speed;
        }else{
          //count = 0;
          stage = 1;
        }}
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
        if (wrongPillar==1){
            leftMotor->setVelocity(15);
            rightMotor->setVelocity(15);
            wrongPillar=0;
            stage=7;
        }
        // taking the left turn
        else {
        // takjing the left turn
        if(leftPsVal < lpos + advancedBy + turn){
          cout<<"*******stopped then turn right*****"<<'\n';
          leftMotor->setVelocity(sharpturn_speed);
          rightMotor->setVelocity(0);
          mleft = sharpturn_speed; mright = 0;
        }else{
          stage = 1;
        }
      }}

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
      if (count < 4){
        leftMotor->setVelocity(mleft);
        rightMotor->setVelocity(mright);
        stage = 1;
      }else{
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        stage = 1;
        count =0;
        break;

      }
    }else if(stage == 20){

      cout << "right = " <<R_DS << "  left = " << L_DS <<'\n';

      double left_wall_error = L_DS -10;
      double right_wall_error = R_DS - 10;

      double left_motor_speed;
      double right_motor_speed;

      if (R_DS < distance_to_wall){
        double wall_offset = kpp*right_wall_error +kdd*(right_wall_error - right_prev_wall);
        right_prev_wall = right_wall_error;
        left_motor_speed = baseSpeed + wall_offset;
        right_motor_speed = baseSpeed - wall_offset;

        leftMotor->setVelocity(Mdriver(left_motor_speed));
        rightMotor->setVelocity(Mdriver(right_motor_speed));

      }else if(L_DS < distance_to_wall){
        double wall_offset = kpp*left_wall_error +kdd*(left_wall_error - left_prev_wall);
        left_prev_wall = left_wall_error;

        left_motor_speed = baseSpeed - wall_offset;
        right_motor_speed = baseSpeed + wall_offset;

        leftMotor->setVelocity(Mdriver(left_motor_speed));
        rightMotor->setVelocity(Mdriver(right_motor_speed));
      }else if(L_DS < distance_to_wall && R_DS < distance_to_wall){
        double wall_offset = kpp*(R_DS - L_DS) + kdd*(R_DS -L_DS - mid_prev_wall);
        mid_prev_wall = R_DS - L_DS;

        left_motor_speed = baseSpeed + wall_offset;
        right_motor_speed = baseSpeed - wall_offset;

        leftMotor->setVelocity(Mdriver(left_motor_speed));
        rightMotor->setVelocity(Mdriver(right_motor_speed));

      }

      else{
        stage = 1;
      }
    } //end of stage 20
    //.......................ramp and pole detection.................................

    else if (stage == 4 && colordif == 2 && finishedcircle==1){
      cout<<"************************stage 4 color dif 2**********************"<<'\n';
      if ((leftPsVal < lpos + advancedonRamp) || (rightPsVal < rpos + advancedonRamp)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);

        //creating a memory to save wheels current speeds
        mleft = forward_speed;  mright = forward_speed;

      }else{

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        mleft = 0; mright = 0;

        if(rightPsVal < rpos + advancedonRamp + turn){
          cout<<"*******stopped then turn left*****"<<'\n';
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(sharpturn_speed);

          mleft = 0; mright = sharpturn_speed;

        }else{
          //count = 0;
          stage = 6;
        }



      }


    }else if (stage == 4 && colordif == 1 && finishedcircle == 1){
      cout<<"************************stage 4 color dif 1**********************"<<'\n';
      if ((leftPsVal < lpos + advancedonRamp) || (rightPsVal < rpos + advancedonRamp)){
        leftMotor->setVelocity(forward_speed);
        rightMotor->setVelocity(forward_speed);
        //creating a memory to save wheels current speeds
        mleft = forward_speed;  mright = forward_speed;
      }else{
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        mleft = 0; mright = 0;

        if(leftPsVal < lpos + advancedonRamp + turn){
          cout<<"*******stopped then turn right*****"<<'\n';
          leftMotor->setVelocity(sharpturn_speed);
          rightMotor->setVelocity(0);

          mleft = sharpturn_speed; mright = 0;
        }else{
          //count = 0;
          stage = 7;

      }


      }

    }

    else if (stage==6){
      double rampSpeed = 3;
      cout<<"************************stage 6 color dif 2**********************"<<noOfPoles<<"colordif"<<colordif<<'\n';
      if (leftMostValue==1 && rightMostValue==0 && noOfPoles==colordif){ // code for robot when poles ae correctly found
        stage = 2;
        detect = true;
        lpos = leftPsVal;
        rpos = rightPsVal;
        cout<<"###left detected#### "<<count<<'\n';
        count = 0;}
        else if (leftMostValue==1 && rightMostValue==0 && noOfPoles!=colordif){ // code for robot when poles are not correctly found
          lpos = leftPsVal;
          rpos = rightPsVal;
          noOfPoles = 0;
          cout<<"###wrong turn#### "<<count<<'\n';
          stage = 30;
        }
        else{
          //.........function for pillar detecting..........

          double R_DS=right_ultrasound->getValue();
          double L_DS=left_ultrasound->getValue();
          cout <<"LS- "<<L_DS<<"      RS- "<<R_DS<< endl;

          if(L_DS<=15.0 && flagPillar==false){
            noOfPoles+=1;
            flagPillar = true;
          }
          else if (L_DS>15.0 && flagPillar==true){
            flagPillar=false;
          }
          else{
            count = 0;

            double offset = PID_calc(); //get the offset by calling pre defined function

            //---------------------set motor speed values to minimize the error------------------------

            double left = rampSpeed + offset;
            double right = rampSpeed - offset;

            //---call a function to map the above speds within its maximum & minimum speed---

            double leftSpeed = Mdriver(left);
            double rightSpeed = Mdriver(right);


            mleft = leftSpeed;
            mright = rightSpeed;

            //----------------------pass the speeds to the motor for run------------------------------

            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);


            //-------------print the sensor outputs from the IR array & current offset-----------------
            cout<<"ir0 = "<<sensorValues[0]<<"  ";
            cout<<"ir1 = "<<sensorValues[1]<<"  ";
            cout<<"ir2 = "<<sensorValues[2]<<"  ";
            cout<<"ir3 = "<<sensorValues[3]<<"  ";
            cout<<"ir4 = "<<sensorValues[4]<<"  ";
            cout<<"ir5 = "<<sensorValues[5]<<"  ";
            cout<<"ir6 = "<<sensorValues[6]<<"  ";
            cout<<"ir7 = "<<sensorValues[7]<<'\n';

            cout<<" offset : "<<offset<<'\n';
          }}
        }
        else if (stage==7){
          double rampSpeed = 3;
          cout<<"************************stage 7 color dif 1**********************"<<noOfPoles<<"colordif"<<colordif<<'\n';
          if (leftMostValue==0 && rightMostValue==1 && noOfPoles==colordif){

            stage = 3;
            detect = true;
            lpos = leftPsVal;
            rpos = rightPsVal;
            cout<<"###right detected ramp#### "<<count<<'\n';
            count = 0;}
            else if (leftMostValue==0 && rightMostValue==1 && noOfPoles!=colordif){
              lpos = leftPsVal;
              rpos = rightPsVal;
              cout<<"###wrong turn#### "<<count<<'\n';
              noOfPoles = 0;
              stage = 8;
            }
            else{
              double R_DS=right_ultrasound->getValue();
              double L_DS=left_ultrasound->getValue();
              cout <<"LSramp- "<<L_DS<<"      RS- "<<R_DS<< endl;
              if(R_DS<=15.0 && flagPillar==false){
                noOfPoles+=1;
                flagPillar = true;
              }
              else if (R_DS>15.0 && flagPillar==true){
                flagPillar=false;
              }
              count = 0;

              double offset = PID_calc(); //get the offset by calling pre defined function

              //---------------------set motor speed values to minimize the error------------------------

              double left = rampSpeed + offset;
              double right = rampSpeed - offset;

              //---call a function to map the above speds within its maximum & minimum speed---

              double leftSpeed = Mdriver(left);
              double rightSpeed = Mdriver(right);


              mleft = leftSpeed;
              mright = rightSpeed;

              //----------------------pass the speeds to the motor for run------------------------------

              leftMotor->setVelocity(leftSpeed);
              rightMotor->setVelocity(rightSpeed);


              //-------------print the sensor outputs from the IR array & current offset-----------------
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
          }
          else if(stage==8){

            if ((leftPsVal > lpos - 8) || (rightPsVal < rpos + 8)){  // Needs to calibrate(turn 180)
              leftMotor->setVelocity(-8);
              rightMotor->setVelocity(8);
              cout<<"###180 turn#### "<<count<<'\n';

            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              wrongPillar=1;
              stage=1;
            }
          }
          else if(stage==30){

            if ((leftPsVal < lpos + 8) || (rightPsVal > rpos - 8)){  // Needs to calibrate(turn 180)
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(-8);
              cout<<"###180 turn#### "<<count<<'\n';

            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              wrongPillar=1;
              stage=1;
            }
          }
          //turned right and pillar count is wrong
          else if(stage ==9 && wrongPillar==1){
            if ((leftPsVal < lpos + 5) || (rightPsVal < rpos + 5)){
              leftMotor->setVelocity(8);
              rightMotor->setVelocity(8);

              mleft = 8;
              mright = 8;
            } else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);

              mleft = 0;
              mright = 0;
              wrongPillar=0;
              stage=6;

            }  }

            //turned left and pillar count is wrong
            else if(stage ==10 && wrongPillar==1){
              if ((leftPsVal < lpos + 5) || (rightPsVal < rpos + 5)){
                leftMotor->setVelocity(8);
                rightMotor->setVelocity(8);

                mleft = 8;
                mright = 8;
              } else{
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);

                mleft = 0;
                mright = 0;
                wrongPillar=0;
                stage=7;

              }  }

              cout <<"  count =  "<<count<<'\n';
              cout <<"        "<<'\n';
              //count ++;
            } // end of main while loop

            delete robot;
            return 0;
          }
