//Author : Nikeshi Kumarasinghe
//Modified date : 09/07/2021

// parameters
int gateCount=0;
bool rampCompleted= true;
double sensorValues[10];

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

void frontIR(){
    double frontIrValue = sensorValues[10];
    if (frontIrValue > 800){  // Needs to calibrate Front IR Sensor
        frontIrValue = 1;
    }else{
        frontIrValue = 0;
    }
}


//------------------------Passing gates------------------------
if rampCompleted{
    while (gateCount < 3){
        if (stage == 1){
            //line following code
            leftMotor->setVelocity(8);
            rightMotor->setVelocity(8);
        }
        //detect junction
        else if (stage==4){
            //stop robot
            leftMotor->setVelocity(0);
            rightMotor->setVelocity(0);
            //take reading from IR sensor
            read();
            frontIR();
            if (frontIrValue==1){
                gateCount=gateCount+1;
                if (frontIrValue==0){
                    break;
                }
            }
        }
    }

} 