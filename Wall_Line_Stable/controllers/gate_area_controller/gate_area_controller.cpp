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
double additional = 2;
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
int state =0; // after the circular area
float advancedonRamp = 1.5;
int noOfPoles=0;
bool flagPillar=false;
int colordif=2;
int wrongPillar=0;
int finishedcircle=1;
bool ramp_over = false;

// variables for circular maze
int circular = 0; // Initially circular algorithm is not activated
bool first_exit = false;
bool second_exit = false;
bool third_exit = false;
float reverse = 3;
bool box_detected = false;
double SIR_threshold = 50;
bool flag1_const_dist_to_box = false;
bool flag2_const_dist_to_box = false;


// variables for gate area
int gate_count = 0;
int gate = 0;

// variables for color detection
double linear = 0.0;
double end_position = -0.16; 
bool flag1 = true;
double arm_ps_val;


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

  DistanceSensor *sharp_IR = robot->getDistanceSensor("middle");
  sharp_IR->enable(TIME_STEP);
  
  // cloor detection
  Motor *linear_actuator;
  linear_actuator=robot->getMotor("linear motor");
  Motor *arm_motor;
  arm_motor=robot->getMotor("arm_motor");
  arm_motor->setPosition(INFINITY);
  arm_motor->setVelocity(0);
  
  
  PositionSensor *arm_ps = robot->getPositionSensor("arm_ps");
  arm_ps->enable(TIME_STEP);

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

    // maze related
    double sharp_ir_value = sharp_IR->getValue();
    double sharp_ir = sharp_ir_value;

    if (sharp_ir_value < 600){  // Needs to calibrate Front IR Sensor
      sharp_ir_value = 1;
    }else{
      sharp_ir_value = 0;
    }

  // color detection
    
    double arm_ps_val = arm_ps->getValue();
    
    
    //print the position value as radians
    // cout<< "Left PS = " << leftPsVal;
    // cout<<"  Right PS = " <<rightPsVal<<'\n';
    // cout<< "Left Most IR = " <<leftMostValue;
    // cout<<"  Right Most IR = " <<rightMostValue<<'\n';
    //cout<<"  sharp IR = " <<sharp_ir_value<<'\n';


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
            }else if(sensorValues[2]==1 && sensorValues[3]==1 && sensorValues[4]==1 && sensorValues[5]==1 && ramp_over == true && gate_count <3){
              stage = 580;
              cout << "Gate area started" << '\n'; 
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);        
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
              if (wrongPillar==1 or box_detected == true){
                leftMotor->setVelocity(15);
                rightMotor->setVelocity(15);
                wrongPillar=0;
                stage=6;
                if(box_detected){
                  stage = 4;
                  state = 0;
                  circular = 5;
                  box_detected = false;
                }
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
                  if (circular != 0){
                    stage = 4;
                    state = 0; // for back to circular algo
                  }else{
                    //count = 0;
                    stage = 1;
                  }
                }
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
                  if (circular != 0){
                    stage = 4;
                    state = 0; // for back to circular algo
                  }else{
                    //count = 0;
                    stage = 1;
                  }
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
                  if (circular != 0){
                    stage = 4;
                    state = 0; // for back to circular algo
                  }else{
                    //count = 0;
                    stage = 1;
                  }
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
                  if (circular != 0){
                    stage = 4;
                    state = 0; // for back to circular algo
                  }else{
                    //count = 0;
                    stage = 1;
                  }
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
  //--------------------------------Start Circular algo-------------------------
            }else if (stage == 4 && state == 0){
              cout << "circular = "<< circular << " stage = "<<stage<<" state = "<<state<<"\n";
              if (circular == 0){                 // turn left
                circular = 1;
                state = 1;

              }else if (circular == 1){          // line follow & turn right
                turn  = turn + additional;
                stage = 1;
                circular = 2;
              }else if (circular == 2){         // check the box availability
                turn  = turn - additional;
                if (sharp_ir_value == 1){         // If detected, go 22
                  cout<<"================box detected=========$$$$$$$$$$$$$$$$$$$$$$$"<<"\n";
                  cout<<" "<<"\n";
                  //stage = 2;
                  circular = 22;
                  lpos = leftPsVal;
                  rpos = rightPsVal;
                }else{                          // If not detected, line follow
                  stage = 1;
                  state = 0;
                  circular = 12;
                  first_exit = true;      //////////////////////////////////
                }
              }else if (circular == 22){        // go 10 cm backward
              if ((leftPsVal < lpos + 8.2) || (rightPsVal > rpos - 8.2)){  // Needs to calibrate
                  leftMotor->setVelocity(8);
                  rightMotor->setVelocity(-8);
                  cout << "@@@@@@@@@@@ 180 turn @@@@@@@@@@@@@@@@@@@@"<< '\n';

                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  stage = 4;
                  state = 0;
                  circular = 23;
                  lpos =  leftPsVal;  rpos =  rightPsVal;
                }
                // if ((leftPsVal > lpos - reverse) || (rightPsVal > rpos - reverse)){  // Needs to calibrate
                  // leftMotor->setVelocity(-8);
                  // rightMotor->setVelocity(-8);

                // }else{
                  // leftMotor->setVelocity(0);
                  // rightMotor->setVelocity(0);
                  // circular = 3;
                  // stage = 2;                // turn left
                // }
              }else if(circular == 23){
                if ((leftPsVal > lpos - reverse) || (rightPsVal > rpos - reverse)){  // Needs to calibrate
                  leftMotor->setVelocity(-8);
                  rightMotor->setVelocity(-8);


                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  circular = 3;
                  state = 2;
                  stage = 1;
                  box_detected = true;                // turn left
                }
              }else if (circular == 3){         // line follow and turn left
                stage = 1;
                circular = 4;
              }else if (circular == 4){         // turn right
                stage = 3;
                circular = 5;
              }else if (circular == 5){         // line follow & turn right
                stage = 1;
                circular = 6;
              }else if (circular == 6){        // line follow & turn right
                stage = 1;
                circular = 7;
                state = 2;
                lpos = leftPsVal;
                rpos = rightPsVal;
              }else if (circular == 7){       // Go 10 cm straight
                if (sharp_ir > SIR_threshold && !flag2_const_dist_to_box){  // Needs to calibrate
                cout << "======Moved forward======" << '\n';
                  leftMotor->setVelocity(8);
                  rightMotor->setVelocity(8);
                  flag1_const_dist_to_box = true;

                }else if(sharp_ir < SIR_threshold && !flag1_const_dist_to_box){
                  cout << "======Moved backward======" << '\n';
                  leftMotor->setVelocity(-8);
                  rightMotor->setVelocity(-8);
                  flag2_const_dist_to_box = true;
                  
                }else{
                  flag1_const_dist_to_box = false;
                  flag2_const_dist_to_box = false;
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);

                  //------------------------------------------------

                  // Color detection Algorithm
                  circular = 25;
                  stage = 4;
                  state = 0;

                  //------------------------------------------------

                  // circular = 8;
                  // stage = 4;
                  // state = 0;
                  // lpos = leftPsVal;
                  // rpos = rightPsVal;
                }
              }else if (circular == 8){         // turn 180 degree
                if ((leftPsVal < lpos + 8.2) || (rightPsVal > rpos - 8.2)){  // Needs to calibrate
                  leftMotor->setVelocity(8);
                  rightMotor->setVelocity(-8);

                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  if (first_exit == false){
                    circular = 9; // previously in case 1 this was 9
                    stage = 4;
                    state = 0;
                    first_exit = true;
                    lpos = leftPsVal;
                    rpos = rightPsVal;
                  }else{
                    if (second_exit == false){
                      lpos =  leftPsVal;  rpos =  rightPsVal;
                      circular = 13;
                      stage = 4;
                      state = 0;
                      second_exit = true;
                    }else{
                      if (third_exit == false){
                        circular = 17;
                        stage = 4;
                        state = 0;
                        third_exit = true;
                        lpos = leftPsVal;
                        rpos = rightPsVal;
                      }else{
                        circular = 19;
                        stage = 4;
                        state = 0;
                        lpos = leftPsVal;
                        rpos = rightPsVal;
                      }
                    }
                  }
                }
              }else if (circular == 9){       // line follow & turn left
                if ((leftPsVal > lpos - 1.7*reverse ) || (rightPsVal > rpos - 1.7*reverse)){  // Needs to calibrate
                  leftMotor->setVelocity(-8);
                  rightMotor->setVelocity(-8);


                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  circular = 10;
                  state = 1;
                  stage = 1;
                  //box_detected = true;                // turn left
                }
                // stage = 1;
                // circular = 10; // prev: in ca1 this was 10
                // state = 1; // previouly in case 1 this was 1
              }else if (circular == 10){     // line follow & turn left
                stage = 1;
                circular = 11;
                state = 1;
              }else if (circular == 21){     // line follow & turn left
                stage = 1;
                circular = 0;
                state = 3;               //----------End of circular(3)
              }else if (circular == 11){     // line follow & turn right
                stage = 1;
                circular = 0;
                state = 3;               //----------End of circular(1)
                cout << "=======================End of the maze=========================" << '\n';                
              }else if (circular == 12){     // check the box availability
                if (sharp_ir_value == 1){         // If detected, go 10 cm straight
                  stage = 4;
                  circular = 7;
                  state = 0;
                  lpos =  leftPsVal;  rpos =  rightPsVal;
                }else{                          // If not detected, turn left
                  stage = 4;
                  state = 1;
                  circular = 16;
                  second_exit = true;      //////////////////////////
                }
              }else if (circular == 13){        // go 20 cm straight
                if ((leftPsVal < lpos + 4) || (rightPsVal < rpos + 4)){  // Needs to calibrate
                  leftMotor->setVelocity(8);
                  rightMotor->setVelocity(8);

                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  circular = 14;
                  stage = 4;
                  state = 0;
                }
              }else if (circular == 14){        // line follow & turn right
                stage = 1;
                state = 2;
                circular = 15;
              }else if (circular == 15){       // line follow & turn left
                stage = 1;
                circular = 0;
                state = 3;              //----------End of circular(2)
                cout << "=======================End of the maze=========================" << '\n';
              }else if (circular == 16){      // check the box availability
                if (sharp_ir_value == 1){          // If detected, go 10 cm straight
                  stage = 4;
                  circular = 7;
                  state = 0;
                  lpos =  leftPsVal;  rpos =  rightPsVal;
                }else{                      // If not detected, go circular 18
                  stage = 4;
                  lpos =  leftPsVal;  rpos =  rightPsVal;
                  circular = 18;
                  state = 0;
                  third_exit = true;    //////////////////////////
                }
              }else if (circular == 17){       // line follow & turn right
              if ((leftPsVal > lpos - 1.7*reverse ) || (rightPsVal > rpos - 1.7*reverse)){  // Needs to calibrate
                  leftMotor->setVelocity(-8);
                  rightMotor->setVelocity(-8);


                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  stage = 1;
                circular = 14;
                state = 2;
                  //box_detected = true;                // turn left
                }
                // stage = 1;
                // circular = 14;
                // state = 2;
              }else if (circular == 18){       // turn 180 degree
                if ((leftPsVal < lpos + 8.2) || (rightPsVal > rpos - 8.2)){  // Needs to calibrate
                  leftMotor->setVelocity(8);
                  rightMotor->setVelocity(-8);
                  cout << "@@@@@@@@@@@ 180 turn @@@@@@@@@@@@@@@@@@@@"<< '\n';

                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  stage = 4;
                  state = 0;
                  circular = 7;
                  lpos =  leftPsVal;  rpos =  rightPsVal;
                }
              }else if (circular == 19){       // line follow & turn left
                if ((leftPsVal > lpos - 1.7*reverse ) || (rightPsVal > rpos - 1.7*reverse)){  // Needs to calibrate
                  leftMotor->setVelocity(-8);
                  rightMotor->setVelocity(-8);


                }else{
                  leftMotor->setVelocity(0);
                  rightMotor->setVelocity(0);
                  stage = 1;
                circular = 14;
                state = 1;
                  //box_detected = true;                // turn left
                }
                // stage = 1;
                // circular = 14;
                // state = 1;
              }else if(circular == 25){
//==================== box manipulation==================
              // linear_actuator, end_position
              
              if (arm_ps_val < 2.19){
                arm_motor->setVelocity(1.5);
              }else{
                arm_motor->setVelocity(0);
                circular = 26;
              }
              //--------------------------------------
              }else if (circular == 26){
              if(linear > end_position && flag1){
                  linear_actuator->setPosition(linear);
                  linear -= 0.02;
                  cout << linear << '\n';      
              }else if(linear <= 0){
                  flag1 = false;
                  linear_actuator->setPosition(linear);
                  linear += 0.02; 
                  cout << linear << '\n';
              }else{
                  circular = 27;
              }
              }else if (circular == 27){
              if (arm_ps_val > -1.57){
                arm_motor->setVelocity(-1.5);
              }else{
                arm_motor->setVelocity(0);
                circular = 8;
                stage = 4;
                state = 0;
                lpos = leftPsVal;
                rpos = rightPsVal;
              }
              }
            }
            //--------------------------------End of Circular algo--------------------------------------------

            else if(stage == 20){

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
              if (leftMostValue==1 && rightMostValue==0 && noOfPoles==colordif ){ // code for robot when poles ae correctly found
                stage = 2;
                detect = true;
                ramp_over = true;
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
                    ramp_over = true;
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
//============================ Gate area algorithm ===========================
                      else if (stage == 580){
                        if(gate == 0){
                          if(sharp_ir_value == 1){
                           gate = 1;
                          }
                        }else if(gate ==1){
                          if(sharp_ir_value == 0){
                          gate = 2;
                          lpos =  leftPsVal;
                          rpos =  rightPsVal;}
                                                 
                        }else if(gate == 2){
                          if ((leftPsVal < lpos + 2) || (rightPsVal < rpos + 2)){  // Needs to calibrate
                          leftMotor->setVelocity(8);
                          rightMotor->setVelocity(8);

                          }else{
                            leftMotor->setVelocity(0);
                            rightMotor->setVelocity(0);
                            gate_count++;
                            stage = 1;
                            gate =0;
                           }
                        }
                        
                        }
                        // cout << "Sharp IR at + point: " << sharp_ir << '\n';
                        // leftMotÇor->setVelocity(0);
                        // rightMotor->setVelocity(0);
                        // if(sharp_ir_value == 1){
                           // gate_count++;
                           
                          // if(sharp_ir_value == 0){
                          // leftMotor->setVelocity(2);
                          // rightMotor->setVelocity(2);
                          // stage ==1;
                          // }
                        // }
                      

                      // cout <<"  count =  "<<count<<'\n';
                      // cout <<"        "<<'\n';
                      //count ++;
                      
                      cout << "arm position = " << arm_ps_val << '\n';
                    } // end of main while loop

                    delete robot;
                    return 0;
                  }
