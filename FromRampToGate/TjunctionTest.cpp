

//Variables for pillar detecting
int noOfPoles=0;
bool flagPillar=false;
int colordif=2;
int wrongPillar=0;
int finishedcircle=1;

    
    //.......................ramp and pole detection.................................

          else if (stage == 4 && colordif == 2 && finishedcircle==1){
          std::cout<<"************************stage 4 color dif 2**********************"<<std::endl;
            if ((leftPsVal < lpos + 1.5) || (rightPsVal < rpos + 1.5)){
              leftMotor->setVelocity(5);
              rightMotor->setVelocity(5);
              
              mleft = 5;
              mright = 5;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[4]==0){
                std::cout<<"*******stopped then turn left*****"<<std::endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(5);
                
                mleft = 0;
                mright = 5;
              }else{
                //count = 0;
                stage = 6;
              }
                
              
            }
          
          
          }else if (stage == 4 && colordif == 1 && finishedcircle == 1){
            std::cout<<"************************stage 4 color dif 1**********************"<<std::endl;
            if ((leftPsVal < lpos + 1.5) || (rightPsVal < rpos + 1.5)){
              leftMotor->setVelocity(5);
              rightMotor->setVelocity(5);
              
              mleft = 5;
              mright = 5;
            }else{
              leftMotor->setVelocity(0);
              rightMotor->setVelocity(0);
              
              mleft = 0;
              mright = 0;
              if (sensorValues[4]==0){
                std::cout<<"*******stopped then turn right*****"<<std::endl;
                leftMotor->setVelocity(5);
                rightMotor->setVelocity(0);
                
                mleft = 5;
                mright = 0;
              }else{
                //count = 0;
                stage = 7;
              }
                
              
            }
          
          }
          
          else if (stage==6){
            double baseSpeed = 1;
            std::cout<<"************************stage 6 color dif 2**********************"<<noOfPoles<<"colordif"<<colordif<<std::endl;
            if (leftMostValue==1 && rightMostValue==0 && noOfPoles==colordif){ // code for robot when poles ae correctly found
              baseSpeed = 5;
              stage = 2;
              detect = true;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###left detected#### "<<count<<std::endl;
              count = 0;}
            else if (leftMostValue==1 && rightMostValue==0 && noOfPoles!=colordif){ // code for robot when poles are not correctly found
              lpos = leftPsVal;
              rpos = rightPsVal;
              noOfPoles = 0;
              std::cout<<"###wrong turn#### "<<count<<std::endl;
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
              }}
            }
          else if (stage==7){
            double baseSpeed = 1;
            std::cout<<"************************stage 7 color dif 1**********************"<<noOfPoles<<"colordif"<<colordif<<std::endl;
            if (leftMostValue==0 && rightMostValue==1 && noOfPoles==colordif){
              baseSpeed = 5;
              stage = 3;
              detect = true;
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###right detected ramp#### "<<count<<std::endl;
              count = 0;}
            else if (leftMostValue==0 && rightMostValue==1 && noOfPoles!=colordif){
              lpos = leftPsVal;
              rpos = rightPsVal;
              std::cout<<"###wrong turn#### "<<count<<std::endl;
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
            }
            else if(stage==8){
   
              if ((leftPsVal > lpos - 8) || (rightPsVal < rpos + 8)){  // Needs to calibrate(turn 180)
                leftMotor->setVelocity(-8);
                rightMotor->setVelocity(8);
                std::cout<<"###180 turn#### "<<count<<std::endl;

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
                std::cout<<"###180 turn#### "<<count<<std::endl;

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

