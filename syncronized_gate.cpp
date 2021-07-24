//Author : Nikeshi Kumarasinghe
//Modified date : 09/07/2021
#define TIME_STEP 1000
// parameters gate
int gateCount=0;
int gateCompleted=0;
int rampCompleted= 1;
int delay=0;
double sensorValues[10];

//----function to read the values of the sensors and convert to binary-------


DistanceSensor *sharp_IR= robot->getDistanceSensor("fnt");
double sharp_ir_value= Sharp_IR->getValue();
double  leftMostValue = sensorValues[3];
double  rightMostValue = sensorValues[4];

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
}

if (frontIrValue < 500){  
    frontIrValue = 1;
}else{
    frontIrValue = 0;
}



//------------------------Passing gates------------------------

if (gateCount < 3 && rampCompleted==1 && gateCompleted==0){
    if (leftMostValue==1 && rightMostValue==1 ){
        cout<<"#### plus point detected #### ";
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        mleft = 0;mright = 0;
        
        if (frontIrValue==0){
            leftMotor->setVelocity(0);
            rightMotor->setVelocity(0);
            mleft = 0;mright = 0;
            if (frontIrValue==1){
                gateCount=gateCount+1;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                mleft = 0;mright = 0;
                if (frontIrValue==1){
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    mleft = 0;mright = 0;
                }
                else{
                    stage=1;
                }
            }

        }
        else if(frontIrValue==1){
            gateCount=gateCount+1;
            leftMotor->setVelocity(0);
            rightMotor->setVelocity(0);
            mleft = 0;mright = 0;
            if (frontIrValue==0){
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                mleft = 0;mright = 0;
            }
            else{
                stage=1
            }

        }

    }else if (gateCount==2){
        gateCompleted=1;
    }else{
        stage=1;
        state = 0;
    }
    
}

