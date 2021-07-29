// Authors        : Team BRAND
// Modified date  : 29/07/2021

//==============================================================================
//*                                  Preamble                                  *
//==============================================================================

//Adding required libraries
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace webots;
using namespace std;

// defining variables
#define TIME_STEP 16
#define MAX_SPEED 10

// start and end of the overall task
bool beginTask = false;
bool endTask = false;
double freelyForward = 4;

double baseSpeed = 4;
double lastErrorLF = 0;
double setPositionLF = 3500;
double IRsensorValues[10];
double leftWheelPSValue;
double rightWheelPSValue;
double turnValue = 8.5;
double additionalturnValue = 2;


int count1 = 0; // for dotted line

//variables for wall following
float kpOfWF = 0.5;
float kdOfWF = 0.1;
double rightPrevWall = 0;
double leftPrevWall = 0;
double midPrevWall = 0;
double distanceToWall = 16;


// Variables related to take turns and wheels
float advancedBy = 0.7;    // distanceToWall of free move when a junction is detected.
float forwardSpeed = 4;    // free moving speed
float sharpturnSpeed = 4;  // speed of taking turns
double leftWheelPrevSpeed;
double rightWheelPrevSpeed;

// Variables related to state of the task
int stage = 1;
int count2 = 0;
int detectingTurn = 2;

//Variables for pillar detecting
int state = 0; // after the circular area
float advancedonRamp = 1.5; // distance to move forward at the top of the ramp
int noOfPoles = 0;
bool flagPillar = false; // if a pillar is detected, make this flag true
int colorDiff = 1; // absolute difference between detected front and top colors
int wrongPillar = 0; // if the count of pillars is wrong, make this flag 1
int finishedcircle = 1;
bool rampOver = false;
double distanceToLeftPillars;
double distanceToRightPillars;

// variables for circular maze
int circular = 0; // Initially circular algorithm is not activated
bool first_exit = false;
bool second_exit = false;
bool third_exit = false;
float reverse = 3;
bool box_detected = false;
double distanceToBox = 50; // distance to the box in the unit of sharp ir measure
bool flag1_const_dist_to_box = false;
bool flag2_const_dist_to_box = false;


// variables for gate area
int gateCount = 0;
int gate = 0; // sub stages inside the gate area

// variables for color detection
string colors[4] = { "gray","red","green","blue" };
Camera* camera;
int getColorAt (int x, int y);

//==============================================================================
//*                         Defining custom functions                          *
//==============================================================================

//---------Reading the values of the sensors and convert to binary 1/0----------
void read () {
	for ( int i = 0; i < 10; i++ ) {
		if ( IRsensorValues[i] < 700 ) {
			IRsensorValues[i] = 1;
		} else {
			IRsensorValues[i] = 0;
		}
	}
}
//--------------------------end of the read() function--------------------------

//-------------------------Function for PD calculation--------------------------
double PID_calc () {
	double average = 0;
	double sum = 0;
	for ( int i = 0; i < 8; i++ ) {
		average += IRsensorValues[i] * i * 1000;
		sum += IRsensorValues[i];
	}

	double position = average / sum; // current position
	double kp = 0.008;
	double kd = 0.0002;
	double e = position - setPositionLF; // deviation from the setPositionLF position
	double p = kp * e;
	double d = kd * ( e - lastErrorLF );
	double offset = p + d;
	lastErrorLF = e;
	return offset;
}
//------------------------end of the PID_calc() function------------------------

//--------------------------Function for motor driving--------------------------
double Mdriver (double speed) {

	if ( speed > 0 ) {
		if ( speed > MAX_SPEED ) {
			speed = MAX_SPEED;
		}
	} else {
		if ( speed < -MAX_SPEED ) {
			speed = -MAX_SPEED;
		}
	}
	return speed;
}
//---------------------end of the driver() function----------------------------
// function for getting color

int getColorAt (int x, int y) {
	// x,y specify the point of color extraction
	const unsigned char* image = camera->getImage ();
	int image_width = camera->getWidth ();

	int r = camera->imageGetRed (image, image_width, x, y);
	int g = camera->imageGetGreen (image, image_width, x, y);
	int b = camera->imageGetBlue (image, image_width, x, y);

	int color;
	if ( r > g && r > b ) color = 1;
	if ( g > r && g > b ) color = 2;
	if ( b > g && b > r ) color = 3;

	cout << "Detected color = " << colors[color] << '\n';

	return color;
}


//==============================================================================
//*                                                                            *
//*                        Main function for the robot                         *
//*                                                                            *
//==============================================================================

int main (int argc, char** argv) {

	Robot* robot = new Robot (); // initializing the robot object

	// get a handler to the motors and setPositionLF target position to infinity
	Motor* leftMotor = robot->getMotor ("left motor");
	Motor* rightMotor = robot->getMotor ("right motor");
	leftMotor->setPosition (INFINITY);
	rightMotor->setPosition (INFINITY);

	// defining the initial velocities of the wheels
	leftMotor->setVelocity (0);
	rightMotor->setVelocity (0);

	//initializing the position sensors
	PositionSensor* leftPs = robot->getPositionSensor ("left_ps");
	leftPs->enable (TIME_STEP);
	PositionSensor* rightPs = robot->getPositionSensor ("right_ps");
	rightPs->enable (TIME_STEP);

	// initializing ultrasound sensors
	DistanceSensor* right_ultrasound = robot->getDistanceSensor ("right_ultrasound");
	right_ultrasound->enable (TIME_STEP);
	DistanceSensor* left_ultrasound = robot->getDistanceSensor ("left_ultrasound");
	left_ultrasound->enable (TIME_STEP);

	// initialize infrared sensors
	char sensorNames[10][10] = {
	  "ir0", "ir1", "ir2", "ir3", "ir4","ir5", "ir6", "ir7", // for PID_calc()
	  "leftmost", "rightmost" // for junction detection
	};

// enable the sensors to get measurements
	DistanceSensor* ir[10];
	for ( int i = 0; i < 10; i++ ) {
		ir[i] = robot->getDistanceSensor (sensorNames[i]);
		ir[i]->enable (TIME_STEP);
	}

// initialize front SharpIR sensor
	DistanceSensor* sharp_IR = robot->getDistanceSensor ("middle");
	sharp_IR->enable (TIME_STEP);

	// color detection camera initialization
	camera = robot->getCamera ("color_sensor");
	camera->enable (TIME_STEP);

	//============================================================================
	//*                                 main loop                                *
	//============================================================================

	while ( robot->step (TIME_STEP) != -1 ) {

		// read sensors outputs
		for ( int i = 0; i < 10; i++ ) {
			IRsensorValues[i] = ir[i]->getValue ();
		}

		read (); // call a function to get out put as binary values from the IR array

		//read junction detection sensor values
		double  leftMostValue = IRsensorValues[8];
		double  rightMostValue = IRsensorValues[9];

		//read position sensor values-
		double leftPsVal = leftPs->getValue ();
		double rightPsVal = rightPs->getValue ();

		double distanceToRightWall = right_ultrasound->getValue ();
		double distanceToLeftWall = left_ultrasound->getValue ();

		// getting front SharpIR sensor reading values
		double sharp_ir_value = sharp_IR->getValue ();
		double sharp_ir = sharp_ir_value;

		if ( sharp_ir_value < 600 ) {  // Calibrate Front IR Sensor
			sharp_ir_value = 1;
		} else {
			sharp_ir_value = 0;
		}

	//============================================================================
	//*                            main state machine                           *
	//============================================================================
		if ( stage == 1 ) {
			if ( !beginTask ) {
				if ( ( leftPsVal < freelyForward ) || ( rightPsVal < freelyForward ) ) {  // Needs to calibrate
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (8);
					stage = 1;
					beginTask = false;
					cout << "Move forward to identify the line." << '\n';

				} else {

					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					beginTask = true;
					stage = 1;
					cout << "Identifed the line." << '\n';

				}
			} else if ( leftMostValue == 1 && rightMostValue == 0 ) {
				if ( count2 > detectingTurn ) {
					count2 = 0;
					stage = 2;

					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
					cout << "=========left detected========= " << count1 << '\n';
					count1 = 0;
				} else {
					leftMotor->setVelocity (2);
					rightMotor->setVelocity (2);
					count2 = count2 + 1;
					cout << "=========count2========= " << count2 << '\n';
				}
			} else if ( rightMostValue == 1 && leftMostValue == 0 ) {
				if ( count2 > detectingTurn ) {
					count2 = 0;
					stage = 3;
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
					cout << "right detected" << count1 << '\n';
					count1 = 0;
				} else {
					leftMotor->setVelocity (2);
					rightMotor->setVelocity (2);
					count2 = count2 + 1;
					cout << "=========count2========= " << count2 << '\n';
				}
			} else if ( leftMostValue == 1 && rightMostValue == 1 ) {
				if ( count2 > detectingTurn ) {
					count2 = 0;
					stage = 4;
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
					cout << " T junction detected " << count1 << '\n';
					count1 = 0;
				} else {
					leftMotor->setVelocity (2);
					rightMotor->setVelocity (2);
					count2 = count2 + 1;
					cout << "=========count2========= " << count2 << '\n';
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
				}
			} else if ( IRsensorValues[0] == 0 && IRsensorValues[1] == 0 && IRsensorValues[2] == 0 && IRsensorValues[3] == 0 && IRsensorValues[4] == 0 && IRsensorValues[5] == 0 && IRsensorValues[6] == 0 && IRsensorValues[7] == 0 ) {
				stage = 5;
			} else if ( ( distanceToRightWall <= 15 || distanceToLeftWall <= 15 ) && wrongPillar == 0 ) {
				stage = 20;
				cout << "Wall following started" << '\n';
			} else if ( IRsensorValues[2] == 1 && IRsensorValues[3] == 1 && IRsensorValues[4] == 1 && IRsensorValues[5] == 1 && rampOver == true && gateCount < 3 ) {
				stage = 580;
				cout << "Gate area started" << '\n';
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
			} else {
				count1 = 0;
				double offset = PID_calc (); //get the offset by calling pre defined function

				//setPositionLF motor speed values to minimize the error
				double left = baseSpeed + offset;
				double right = baseSpeed - offset;

				//Call a function to map the above speds within its maximum & minimum speed
				double leftSpeed = Mdriver (left);
				double rightSpeed = Mdriver (right);
				leftWheelPrevSpeed = leftSpeed;
				rightWheelPrevSpeed = rightSpeed;
				//pass the speeds to the motor for run
				leftMotor->setVelocity (leftSpeed);
				rightMotor->setVelocity (rightSpeed);

				//print the sensor outputs from the IR array & current offset
				cout << "ir0 = " << IRsensorValues[0] << "  ";
				cout << "ir1 = " << IRsensorValues[1] << "  ";
				cout << "ir2 = " << IRsensorValues[2] << "  ";
				cout << "ir3 = " << IRsensorValues[3] << "  ";
				cout << "ir4 = " << IRsensorValues[4] << "  ";
				cout << "ir5 = " << IRsensorValues[5] << "  ";
				cout << "ir6 = " << IRsensorValues[6] << "  ";
				cout << "ir7 = " << IRsensorValues[7] << '\n';
				cout << " offset : " << offset << '\n';
			}

		} else if ( stage == 2 ) {
			if ( ( leftPsVal < leftWheelPSValue + advancedBy ) || ( rightPsVal < rightWheelPSValue + advancedBy ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);

				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed; rightWheelPrevSpeed = forwardSpeed;
			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);

				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;
				// advancing is over.
				if ( wrongPillar == 1 or box_detected == true ) { // code to run to ignore left turn at top of ramp  when pillar count is wrong and at circle.

					if ( ( leftPsVal < leftWheelPSValue + freelyForward ) || ( rightPsVal < rightWheelPSValue + freelyForward ) ) {
						leftMotor->setVelocity (8);
						rightMotor->setVelocity (8);

					} else {

						leftMotor->setVelocity (0);
						rightMotor->setVelocity (0);
						wrongPillar = 0;
						stage = 6;
						if ( box_detected ) {
							stage = 4;
							state = 0;
							circular = 5;
							box_detected = false;
						}


					}


				}
			// taking the left turnValue
				else {
					// taking the left turnValue
					if ( rightPsVal < rightWheelPSValue + advancedBy + turnValue ) {
						cout << "*******stopped then turnValue left*****" << '\n';
						leftMotor->setVelocity (0);
						rightMotor->setVelocity (sharpturnSpeed);

						leftWheelPrevSpeed = 0; rightWheelPrevSpeed = sharpturnSpeed;
					} else {
						if ( circular != 0 ) {
							stage = 4;
							state = 0; // for back to circular algo
						} else {
							stage = 1;
						}
					}
				}
			}


		} else if ( stage == 3 ) {
			if ( ( leftPsVal < leftWheelPSValue + advancedBy ) || ( rightPsVal < rightWheelPSValue + advancedBy ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);

				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed;  rightWheelPrevSpeed = forwardSpeed;
			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);

				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;
				// advancing is over.
				if ( wrongPillar == 1 ) { // Code to run at top of ramp to ignore right turn when pillar count is wrong.

					if ( ( leftPsVal < leftWheelPSValue + freelyForward ) || ( rightPsVal < rightWheelPSValue + freelyForward ) ) {
						leftMotor->setVelocity (8);
						rightMotor->setVelocity (8);

					} else {

						leftMotor->setVelocity (0);
						rightMotor->setVelocity (0);
						wrongPillar = 0;
						stage = 7;
					}

				}
			// taking the left turnValue
				else {
					// takjing the left turnValue
					if ( leftPsVal < leftWheelPSValue + advancedBy + turnValue ) {
						cout << "*******stopped then turnValue right*****" << '\n';
						leftMotor->setVelocity (sharpturnSpeed);
						rightMotor->setVelocity (0);
						leftWheelPrevSpeed = sharpturnSpeed; rightWheelPrevSpeed = 0;
					} else {
						if ( circular != 0 ) {
							stage = 4;
							state = 0; // for back to circular algo
						} else {
							stage = 1;
						}
					}
				}
			}

		} else if ( stage == 4 && state == 1 ) {
			if ( ( leftPsVal < leftWheelPSValue + advancedBy ) || ( rightPsVal < rightWheelPSValue + advancedBy ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);

				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed;  rightWheelPrevSpeed = forwardSpeed;

			} else {

				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);

				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;

				if ( rightPsVal < rightWheelPSValue + advancedBy + turnValue ) {
					cout << "*******stopped then turnValue left*****" << '\n';
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (sharpturnSpeed);

					leftWheelPrevSpeed = 0; rightWheelPrevSpeed = sharpturnSpeed;

				} else {
					if ( circular != 0 ) {
						stage = 4;
						state = 0; // for back to circular algo
					} else {
						stage = 1;
					}
				}


			}


		} else if ( stage == 4 && state == 2 ) {
			if ( ( leftPsVal < leftWheelPSValue + advancedBy ) || ( rightPsVal < rightWheelPSValue + advancedBy ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);
				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed;  rightWheelPrevSpeed = forwardSpeed;
			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;

				if ( leftPsVal < leftWheelPSValue + advancedBy + turnValue ) {
					cout << "*******stopped then turnValue right*****" << '\n';
					leftMotor->setVelocity (sharpturnSpeed);
					rightMotor->setVelocity (0);

					leftWheelPrevSpeed = sharpturnSpeed; rightWheelPrevSpeed = 0;
				} else {
					if ( circular != 0 ) {
						stage = 4;
						state = 0; // for back to circular algo
					} else {
						stage = 1;
					}
				}
			}

		} else if ( gateCount == 2 && stage == 4 ) {
			cout << "Task Completed " << '\n';
			if ( ( leftPsVal < leftWheelPSValue + freelyForward ) || ( rightPsVal < rightWheelPSValue + freelyForward ) ) {  // Needs to calibrate
				leftMotor->setVelocity (8);
				rightMotor->setVelocity (8);

			} else {

				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				cout << "Task Completed inside else" << '\n';
				endTask = false;
				break;
			}
		}
	//-----------------------------dotted line area-----------------------------
		else if ( stage == 5 ) {
			count1++;
			if ( count1 < 8 ) { // Going forward when line is not detected.
				leftMotor->setVelocity (leftWheelPrevSpeed);
				rightMotor->setVelocity (rightWheelPrevSpeed);
				stage = 1;
			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				stage = 1;
				count1 = 0;
				break;

			}
		//--------------------------------Start Circular algo-------------------------
		} else if ( stage == 4 && state == 0 ) {
			cout << "circular = " << circular << " stage = " << stage << " state = " << state << "\n";
			if ( circular == 0 ) {                 // turnValue left
				circular = 1;
				state = 1;

			} else if ( circular == 1 ) {          // line follow & turnValue right
				turnValue = turnValue + additionalturnValue;
				stage = 1;
				circular = 2;
			} else if ( circular == 2 ) {         // check the box availability
				turnValue = turnValue - additionalturnValue;
				if ( sharp_ir_value == 1 ) {         // If detected, go 22
					cout << "================box detected=========$$$$$$$$$$$$$$$$$$$$$$$" << "\n";
					cout << " " << "\n";
					circular = 22;
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
				} else {                          // If not detected, line follow
					stage = 1;
					state = 0;
					circular = 12;
					first_exit = true;      //////////////////////////////////
				}
			} else if ( circular == 22 ) {        // go 10 cm backward
				if ( ( leftPsVal < leftWheelPSValue + 8.2 ) || ( rightPsVal > rightWheelPSValue - 8.2 ) ) {  // Needs to calibrate
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (-8);
					cout << "@@@@@@@@@@@ 180 turnValue @@@@@@@@@@@@@@@@@@@@" << '\n';

				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					stage = 4;
					state = 0;
					circular = 23;
					leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
				}

			} else if ( circular == 23 ) {
				if ( ( leftPsVal > leftWheelPSValue - reverse ) || ( rightPsVal > rightWheelPSValue - reverse ) ) {  // Needs to calibrate
					leftMotor->setVelocity (-8);
					rightMotor->setVelocity (-8);


				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					circular = 3;
					state = 2;
					stage = 1;
					box_detected = true;                // turnValue left
				}
			} else if ( circular == 3 ) {         // line follow and turnValue left
				stage = 1;
				circular = 4;
			} else if ( circular == 4 ) {         // turnValue right
				stage = 3;
				circular = 5;
			} else if ( circular == 5 ) {         // line follow & turnValue right
				stage = 1;
				circular = 6;
			} else if ( circular == 6 ) {        // line follow & turnValue right
				stage = 1;
				circular = 7;
				state = 2;
				leftWheelPSValue = leftPsVal;
				rightWheelPSValue = rightPsVal;
			} else if ( circular == 7 ) {       // Go 10 cm straight
				if ( sharp_ir > distanceToBox && !flag2_const_dist_to_box ) {  // Needs to calibrate
					cout << "======Moved forward======" << '\n';
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (8);
					flag1_const_dist_to_box = true;

				} else if ( sharp_ir < distanceToBox && !flag1_const_dist_to_box ) {
					cout << "======Moved backward======" << '\n';
					leftMotor->setVelocity (-8);
					rightMotor->setVelocity (-8);
					flag2_const_dist_to_box = true;

				} else {
					flag1_const_dist_to_box = false;
					flag2_const_dist_to_box = false;
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);

					//------------------------------------------------
					// Color detection Algorithm
					colorDiff = abs (getColorAt (32, 10) - getColorAt (32, 50));
					if ( colorDiff == 0 ) colorDiff = 2;
					cout << "Color Difference = " << colorDiff << '\n';

					//------------------------------------------------

					circular = 8;
					stage = 4;
					state = 0;
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
				}
			} else if ( circular == 8 ) {         // turnValue 180 degree
				if ( ( leftPsVal < leftWheelPSValue + 8.2 ) || ( rightPsVal > rightWheelPSValue - 8.2 ) ) {  // Needs to calibrate
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (-8);

				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					if ( first_exit == false ) {
						circular = 9; // previously in case 1 this was 9
						stage = 4;
						state = 0;
						first_exit = true;
						leftWheelPSValue = leftPsVal;
						rightWheelPSValue = rightPsVal;
					} else {
						if ( second_exit == false ) {
							leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
							circular = 13;
							stage = 4;
							state = 0;
							second_exit = true;
						} else {
							if ( third_exit == false ) {
								circular = 17;
								stage = 4;
								state = 0;
								third_exit = true;
								leftWheelPSValue = leftPsVal;
								rightWheelPSValue = rightPsVal;
							} else {
								circular = 19;
								stage = 4;
								state = 0;
								leftWheelPSValue = leftPsVal;
								rightWheelPSValue = rightPsVal;
							}
						}
					}
				}
			} else if ( circular == 9 ) {       // line follow & turnValue left
				if ( ( leftPsVal > leftWheelPSValue - 1.7 * reverse ) || ( rightPsVal > rightWheelPSValue - 1.7 * reverse ) ) {  // Needs to calibrate
					leftMotor->setVelocity (-8);
					rightMotor->setVelocity (-8);


				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					circular = 10;
					state = 1;
					stage = 1;

				}

			} else if ( circular == 10 ) {     // line follow & turnValue left
				stage = 1;
				circular = 11;
				state = 1;
			} else if ( circular == 21 ) {     // line follow & turnValue left
				stage = 1;
				circular = 0;
				state = 3;               //----------End of circular(3)
			} else if ( circular == 11 ) {     // line follow & turnValue right
				stage = 1;
				circular = 0;
				state = 3;               //----------End of circular(1)
				cout << "=======================End of the maze=========================" << '\n';
			} else if ( circular == 12 ) {     // check the box availability
				if ( sharp_ir_value == 1 ) {         // If detected, go 10 cm straight
					stage = 4;
					circular = 7;
					state = 0;
					leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
				} else {                          // If not detected, turnValue left
					stage = 4;
					state = 1;
					circular = 16;
					second_exit = true;      //////////////////////////
				}
			} else if ( circular == 13 ) {        // go 20 cm straight
				if ( ( leftPsVal < leftWheelPSValue + 4 ) || ( rightPsVal < rightWheelPSValue + 4 ) ) {  // Needs to calibrate
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (8);

				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					circular = 14;
					stage = 4;
					state = 0;
				}
			} else if ( circular == 14 ) {        // line follow & turnValue right
				stage = 1;
				state = 2;
				circular = 15;
			} else if ( circular == 15 ) {       // line follow & turnValue left
				stage = 1;
				circular = 0;
				state = 3;              //----------End of circular(2)
				cout << "=======================End of the maze=========================" << '\n';
			} else if ( circular == 16 ) {      // check the box availability
				if ( sharp_ir_value == 1 ) {          // If detected, go 10 cm straight
					stage = 4;
					circular = 7;
					state = 0;
					leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
				} else {                      // If not detected, go circular 18
					stage = 4;
					leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
					circular = 18;
					state = 0;
					third_exit = true;    //////////////////////////
				}
			} else if ( circular == 17 ) {       // line follow & turnValue right
				if ( ( leftPsVal > leftWheelPSValue - 1.7 * reverse ) || ( rightPsVal > rightWheelPSValue - 1.7 * reverse ) ) {  // Needs to calibrate
					leftMotor->setVelocity (-8);
					rightMotor->setVelocity (-8);


				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					stage = 1;
					circular = 14;
					state = 2;

				}

			} else if ( circular == 18 ) {       // turnValue 180 degree
				if ( ( leftPsVal < leftWheelPSValue + 8.2 ) || ( rightPsVal > rightWheelPSValue - 8.2 ) ) {  // Needs to calibrate
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (-8);
					cout << "@@@@@@@@@@@ 180 turnValue @@@@@@@@@@@@@@@@@@@@" << '\n';

				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					stage = 4;
					state = 0;
					circular = 7;
					leftWheelPSValue = leftPsVal;  rightWheelPSValue = rightPsVal;
				}
			} else if ( circular == 19 ) {       // line follow & turnValue left
				if ( ( leftPsVal > leftWheelPSValue - 1.7 * reverse ) || ( rightPsVal > rightWheelPSValue - 1.7 * reverse ) ) {  // Needs to calibrate
					leftMotor->setVelocity (-8);
					rightMotor->setVelocity (-8);


				} else {
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					stage = 1;
					circular = 14;
					state = 1;

				}

			}

		}
	//--------------------------------End of Circular algo--------------------------------------------

		else if ( stage == 20 ) {

			cout << "right = " << distanceToRightWall << "  left = " << distanceToLeftWall << '\n';

			double left_wall_error = distanceToLeftWall - 10;
			double right_wall_error = distanceToRightWall - 10;

			double left_motor_speed;
			double right_motor_speed;

			if ( distanceToRightWall < distanceToWall ) {
				double wall_offset = kpOfWF * right_wall_error + kdOfWF * ( right_wall_error - rightPrevWall );
				rightPrevWall = right_wall_error;
				left_motor_speed = baseSpeed + wall_offset;
				right_motor_speed = baseSpeed - wall_offset;

				leftMotor->setVelocity (Mdriver (left_motor_speed));
				rightMotor->setVelocity (Mdriver (right_motor_speed));

			} else if ( distanceToLeftWall < distanceToWall ) {
				double wall_offset = kpOfWF * left_wall_error + kdOfWF * ( left_wall_error - leftPrevWall );
				leftPrevWall = left_wall_error;

				left_motor_speed = baseSpeed - wall_offset;
				right_motor_speed = baseSpeed + wall_offset;

				leftMotor->setVelocity (Mdriver (left_motor_speed));
				rightMotor->setVelocity (Mdriver (right_motor_speed));
			} else if ( distanceToLeftWall < distanceToWall && distanceToRightWall < distanceToWall ) {
				double wall_offset = kpOfWF * ( distanceToRightWall - distanceToLeftWall ) + kdOfWF * ( distanceToRightWall - distanceToLeftWall - midPrevWall );
				midPrevWall = distanceToRightWall - distanceToLeftWall;

				left_motor_speed = baseSpeed + wall_offset;
				right_motor_speed = baseSpeed - wall_offset;

				leftMotor->setVelocity (Mdriver (left_motor_speed));
				rightMotor->setVelocity (Mdriver (right_motor_speed));

			}

			else {
				stage = 1;
			}
		} //end of stage 20
		//.......................ramp and pole detection.................................

		else if ( stage == 4 && colorDiff == 2 && finishedcircle == 1 ) {
			cout << "stage 4 color difference = 2" << '\n';
			if ( ( leftPsVal < leftWheelPSValue + advancedonRamp ) || ( rightPsVal < rightWheelPSValue + advancedonRamp ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);

				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed;  rightWheelPrevSpeed = forwardSpeed;

			} else {

				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);

				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;

				if ( rightPsVal < rightWheelPSValue + advancedonRamp + turnValue ) {
					cout << "stopped then turn left on top of ramp" << '\n';
					leftMotor->setVelocity (0);
					rightMotor->setVelocity (sharpturnSpeed);

					leftWheelPrevSpeed = 0; rightWheelPrevSpeed = sharpturnSpeed;

				} else {
					stage = 6;
				}



			}


		} else if ( stage == 4 && colorDiff == 1 && finishedcircle == 1 ) {
			cout << "stage 4 color difference = 1" << '\n';
			if ( ( leftPsVal < leftWheelPSValue + advancedonRamp ) || ( rightPsVal < rightWheelPSValue + advancedonRamp ) ) {
				leftMotor->setVelocity (forwardSpeed);
				rightMotor->setVelocity (forwardSpeed);
				//creating a memory to save wheels current speeds
				leftWheelPrevSpeed = forwardSpeed;  rightWheelPrevSpeed = forwardSpeed;
			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				leftWheelPrevSpeed = 0; rightWheelPrevSpeed = 0;

				if ( leftPsVal < leftWheelPSValue + advancedonRamp + turnValue ) {
					cout << "stopped then turn right on top of ramp" << '\n';
					leftMotor->setVelocity (sharpturnSpeed);
					rightMotor->setVelocity (0);

					leftWheelPrevSpeed = sharpturnSpeed; rightWheelPrevSpeed = 0;
				} else {
					stage = 7;

				}
			}
		} else if ( stage == 6 ) {
			double rampSpeed = 3; //Speed to travel on ramp area
			cout << "stage 6 " << "No: of Poles = " << noOfPoles << "colorDiff = " << colorDiff << '\n';
			if ( leftMostValue == 1 && rightMostValue == 0 && noOfPoles == colorDiff ) { // code for robot when poles are correctly found
				stage = 2;
				rampOver = true;
				leftWheelPSValue = leftPsVal;
				rightWheelPSValue = rightPsVal;
				cout << "left detected " << count1 << '\n';
				count1 = 0;
			} else if ( leftMostValue == 1 && rightMostValue == 0 && noOfPoles != colorDiff ) { // code for robot when poles are not correctly found
				leftWheelPSValue = leftPsVal;
				rightWheelPSValue = rightPsVal;
				noOfPoles = 0;
				cout << "wrong turn" << count1 << '\n';
				stage = 9;
			} else {
			//.........function for pillar detecting..........

				distanceToRightPillars = right_ultrasound->getValue ();
				distanceToLeftPillars = left_ultrasound->getValue ();
				cout << "LeftSensorDistance- " << distanceToLeftPillars << "      RightSensorDistance- " << distanceToRightPillars << endl;

				if ( distanceToLeftPillars <= 15.0 && flagPillar == false ) {  //This is done to avoid count of number of poles increasing when same pole is detected more than once.
					noOfPoles += 1;
					flagPillar = true;
				} else if ( distanceToLeftPillars > 15.0 && flagPillar == true ) {
					flagPillar = false;
				} else {
					count1 = 0;

					double offset = PID_calc (); //get the offset by calling pre defined function

					//---------------------setPositionLF motor speed values to minimize the error------------------------

					double left = rampSpeed + offset;
					double right = rampSpeed - offset;

					//---call a function to map the above speds within its maximum & minimum speed---
					double leftSpeed = Mdriver (left);
					double rightSpeed = Mdriver (right);

					leftWheelPrevSpeed = leftSpeed;
					rightWheelPrevSpeed = rightSpeed;
					//----------------------pass the speeds to the motor for run------------------------------
					leftMotor->setVelocity (leftSpeed);
					rightMotor->setVelocity (rightSpeed);

				}
			}
		} else if ( stage == 7 ) {
			double rampSpeed = 3; //Speed to travel on ramp area
			cout << "stage 7 " << "No: of Poles = " << noOfPoles << "colorDiff = " << colorDiff << '\n';
			if ( leftMostValue == 0 && rightMostValue == 1 && noOfPoles == colorDiff ) {

				stage = 3;
				rampOver = true;
				leftWheelPSValue = leftPsVal;
				rightWheelPSValue = rightPsVal;
				cout << "right detected" << count1 << '\n';
				count1 = 0;
			} else if ( leftMostValue == 0 && rightMostValue == 1 && noOfPoles != colorDiff ) {
				leftWheelPSValue = leftPsVal;
				rightWheelPSValue = rightPsVal;
				cout << "wrong turn" << count1 << '\n';
				noOfPoles = 0;
				stage = 8;
			} else {
				distanceToLeftPillars = left_ultrasound->getValue ();
				distanceToRightPillars = right_ultrasound->getValue ();
				cout << "LeftSensorDistance- " << distanceToLeftPillars << "      RightSensorDistance- " << distanceToRightPillars << endl;
				if ( distanceToRightPillars <= 15.0 && flagPillar == false ) { //This is done to avoid count of number of poles increasing when same pole is detected more than once.
					noOfPoles += 1;
					flagPillar = true;
				} else if ( distanceToRightPillars > 15.0 && flagPillar == true ) {
					flagPillar = false;
				} else {
					count1 = 0;

					double offset = PID_calc (); //get the offset by calling pre defined function

					//---------------------setPositionLF motor speed values to minimize the error------------------------

					double left = rampSpeed + offset;
					double right = rampSpeed - offset;

					//---call a function to map the above speds within its maximum & minimum speed---

					double leftSpeed = Mdriver (left);
					double rightSpeed = Mdriver (right);

					leftWheelPrevSpeed = leftSpeed;
					rightWheelPrevSpeed = rightSpeed;

					//----------------------pass the speeds to the motor for run------------------------------
					leftMotor->setVelocity (leftSpeed);
					rightMotor->setVelocity (rightSpeed);

				}
			}
		}
	// Need two 180 degree turn functions as we need to turn robot 180 degrees in two different dirrections.
	// Else robot will collide with the pillars. As they are too close to L-Junction.
		else if ( stage == 8 ) {

			if ( ( leftPsVal > leftWheelPSValue - 8 ) || ( rightPsVal < rightWheelPSValue + 8 ) ) {  //turning 180 degree when pillar count is wrong.
				leftMotor->setVelocity (-8);
				rightMotor->setVelocity (8);
				cout << "180 turn " << '\n';

			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				wrongPillar = 1;
				stage = 1;
			}
		} else if ( stage == 9 ) { //turning 180 degree when pillar count is wrong.

			if ( ( leftPsVal < leftWheelPSValue + 8 ) || ( rightPsVal > rightWheelPSValue - 8 ) ) {
				leftMotor->setVelocity (8);
				rightMotor->setVelocity (-8);
				cout << "180 turn " << '\n';

			} else {
				leftMotor->setVelocity (0);
				rightMotor->setVelocity (0);
				wrongPillar = 1;
				stage = 1;
			}
		}

	//============================ Gate area algorithm ===========================
		else if ( stage == 580 ) {
			cout << "Execute gate area" << '\n';
			if ( gate == 0 ) {

				if ( sharp_ir_value == 1 ) {
					gate = 1;
				}
			} else if ( gate == 1 ) {

				if ( sharp_ir_value == 0 ) {
					gate = 2;
					leftWheelPSValue = leftPsVal;
					rightWheelPSValue = rightPsVal;
				}

			} else if ( gate == 2 ) {

				if ( ( leftPsVal < leftWheelPSValue + 2 ) || ( rightPsVal < rightWheelPSValue + 2 ) ) {
					leftMotor->setVelocity (8);
					rightMotor->setVelocity (8);

				} else {

					leftMotor->setVelocity (0);
					rightMotor->setVelocity (0);
					gateCount++;
					stage = 1;
					gate = 0;
					endTask = true;
				}
			}

		}

	} // end of main while loop

	delete robot;
	return 0;
}
