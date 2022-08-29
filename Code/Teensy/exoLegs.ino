#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "Joint.h"

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//read only and #define
#define DEBUG 1
//#define GUI
//these are now pins on odrive
#define rightPosPinHip 5  //pin to read pot val for hip pos
#define rightPosPinKnee 4  //pin to read pot val for knee pos
#define rightPosPinAnkle 5  //pin to read pot val for ankle pos
#define leftPosPinHip 6  //pin to read pot val for hip pos
#define leftPosPinKnee 5  //pin to read pot val for knee pos
#define leftPosPinAnkle 4  //pin to read pot val for ankle pos

//const float potValToDeg = 0.009;  //convert pot voltage to degrees
const float homePotVal = 1.62;  //value of joint pot when joint angle == 0, in volts
const int rightHipOffset = 0;  //pot voltage when knee is at 0deg
const int rightKneeOffset = 1.71;  //pot voltage when knee is at 0deg
const int rightAnkleOffset = 459;  //pot voltage when knee is at 0deg
const int leftHipOffset = 0;  //pot voltage when knee is at 0deg
const int leftKneeOffset = 989;  //pot voltage when knee is at 0deg
const int leftAnkleOffset = 459;  //pot voltage when knee is at 0deg
const float posDiffThreshold = 0.02;  //how close to target voltage before its ready, in volts or revs
const float motorCurrentLimit = 10.0;  //max motor current
const int numJoints = 1;  //number of active joints, used for testing

const float stateChangeTime = 1.0 / 799.0;  //time to wait for odrive axis to change states
int led = 13;
int ledState = LOW;

//program advancement variables
bool programRunning = false;  //is program running?
int leftLegTarget = 24;  //start array index for left leg joints
int rightLegTarget = 0;  //start array index for right leg joints
bool allJointsReady = false;  //are all joints at target angle?
bool allJointsHomed = false;  //homing sequence completed for all joints

//serial reading
char inBytes[32];  //array to read in commands via serial, could probably be smaller
bool doneReading = false;  //finished reading cmd via serial?
float inVal = 0;  //manual pos cmd via serial
String optionString = "Options: <x> to clear errors, <c1> to calibrate axis1, <h> to home joints, <p> to start position, <r> to run program, <s> to stop";

//data transfer
String dataToSend = "";
String errorString = "";
unsigned long contactTimer = 0;

//speed & weight variables
float segTime = 0.0314;  //0.0314 time in sec between angle datapoints, effects walk speed
float weight = 53.0;  //body weight in kg, measured by current draw
float bodyWeight = 50.0;  //% body weight supported by lift

//position variables
/*int hipMin = 100;  //in counts: -120000;
int hipMax = 150;  //in counts: 40000;
int kneeMin = 100;  //in counts: -6667;
int kneeMax = 150;  //in counts: 120000;
int ankleMin = 570;  //in counts: -33333;
int ankleMax = 250;  //in counts: 26666;
*/

//error variables
uint64_t errorVals[3] = {0, 0, 0};  //initialize errors to 0, no errors
uint32_t axisError = 0;  //32bit axis error
uint64_t motorError = 0;  //64bit motor error
uint32_t encoderError = 0;  //32bit encoder error
uint32_t currentState = 1;  //1 == IDLE
bool isError = false;  //is there currently an error?

//positive angle is flexion, negative is extension
float hipAngles[] = {19.33, 18.92, 18.45, 17.94, 17.3, 16.4, 15.18, 13.67, 11.97, 10.21, 8.48, 6.74, 4.94, 3.13, 1.42,
                     -0.13, -1.54, -2.87, -4.12, -5.3, -6.4, -7.43, -8.39, -9.27, -10.02, -10.61, -10.95, -10.91, -10.31,
                     -9.0, -6.95, -4.25, -1.05, 2.42, 5.93, 9.22, 12.11, 14.55, 16.53, 18.13, 19.45, 20.54, 21.38, 21.84,
                     21.87, 21.5, 20.84, 20.09, 19.5, 19.18, 19.01
                    };
float kneeAngles[] = {3.97, 7.0, 10.52, 14.12, 17.38, 19.84, 21.27, 21.67, 21.22, 20.20, 18.86, 17.35, 15.73, 14.08, 12.5,
                      11.09, 9.91, 8.97, 8.28, 7.86, 7.72, 7.94, 8.6, 9.76, 11.5, 13.86, 16.97, 20.96, 26.0, 32.03, 38.74,
                      45.6, 52.05, 57.54, 61.66, 64.12, 64.86, 63.95, 61.59, 57.97, 53.27, 47.58, 40.94, 33.46, 25.38,
                      17.27, 9.94, 4.31, 1.12, 0.54, 2.21
                     };
float ankleAngles[] = {0.02, -2.06, -3.88, -4.6, -3.98, -2.4, -0.45, 1.45, 3.04, 4.27, 5.13, 5.71, 6.1, 6.43, 6.76, 7.12,
                       7.54, 7.99, 8.44, 8.86, 9.23, 9.51, 9.62, 9.43, 8.7, 7.2, 4.69, 1.15, -3.26, -8.17, -13.05, -17.13,
                       -19.52, -19.77, -18.12, -15.29, -12.04, -8.85, -5.96, -3.51, -1.64, -0.5, -0.07, -0.16, -0.42, -0.52,
                       -0.26, 0.36, 1.0, 1.2, 0.58
                      };
float *hipAnglesPtr = hipAngles;  //pointer to float array
float *kneeAnglesPtr = kneeAngles;  //pointer to float array
float *ankleAnglesPtr = ankleAngles;  //pointer to float array


ODriveTeensyCAN odriveCAN(1000000);  //CAN bus object
ODriveTeensyCAN *odriveCANPtr = &odriveCAN;  //pointer to CAN bus object

//joint objects
Joint leftKnee(1, leftPosPinKnee, leftKneeOffset, kneeAnglesPtr, 24, odriveCANPtr);  //CAN node_id, potPin, offset, angles array, leg starting target, CANbus
Joint leftAnkle(0, leftPosPinAnkle, leftAnkleOffset, ankleAnglesPtr, 24, odriveCANPtr);
Joint rightKnee(2, rightPosPinKnee, rightKneeOffset, kneeAnglesPtr, 0, odriveCANPtr);
Joint rightAnkle(3, rightPosPinAnkle, rightAnkleOffset, ankleAnglesPtr, 0, odriveCANPtr);
Joint leftHip(4, leftPosPinHip, leftHipOffset, hipAnglesPtr, 24, odriveCANPtr);
Joint rightHip(5, rightPosPinHip, rightHipOffset, hipAnglesPtr, 0, odriveCANPtr);

//pointers to joint objects
Joint *leftKneeR = &leftKnee;
Joint *leftAnkleR = &leftAnkle;
Joint *rightKneeR = &rightKnee;
Joint *rightAnkleR = &rightAnkle;
Joint *leftHipR = &leftHip;
Joint *rightHipR = &rightHip;

Joint *allJoints[] = {leftAnkleR, leftKneeR, leftHipR, rightAnkleR, rightKneeR, rightHipR};  //array of pointers to joint objects

//names corresponding to joint objects. Must be in the same order as allJoints[]
const char *jointNames[] = {"Left ankle", "Left knee", "Left hip", "Right ankle", "Right knee", "Right hip"};  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //start Serial monitor
  pinMode(led, OUTPUT);  //led pin as output
  digitalWrite(led, HIGH);  //turn on led
  ledState = HIGH;
  
  while(!Serial) {
    //wait for computer serial connection
  }
  delay(100);  //brief pause

  #ifdef DEBUG  //debug var to help see code stages
  Serial.println("hey1");
  for(int i = 0;i < numJoints;i++) {  //step through array of joints
    Serial << odriveCAN.Heartbeat() << '\n';  //print heartbeat
  }
  #endif

  delay(100);  //brief pause
  /*for(int i = 0;i < numJoints;i++) {  //step through array of joints
    allJoints[i]->ClosedLoopCtrl();  //put joint into closed loop control
    delay(stateChangeTime);  //8kHz ctrl loop on odrive. Pause for 1 ctrl loop to give joint enough time to enter closed loop control
    
    if(allJoints[i]->GetCurrentState() == ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) {  //check to see if joint actually made it into closed loop
      #ifdef GUI
        //do nothing
      #else
        Serial << jointNames[i] << " ready" << '\n';  //if yes, print that its ready
      #endif
    } else {
      #ifdef GUI
        checkJointErrors(allJoints[i]);  //check joint for errors
        if(isError) {  //if an error was found
          Serial << '<' << errorString << i << '>' << '\n';  //send errors + joint index wrapped in start/end chars
        }
      #else
        Serial << jointNames[i] << " failed to enter closed loop control" << '\n';  //if not, notify that it didnt
        checkJointErrors(allJoints[i]);  //check joint for errors, print them
      #endif
    }
    #ifdef GUI
      errorString = "< , , , ,6>";  //signal end of joint errors
      Serial << errorString << '\n';  //send it
    #endif
    delay(100);  //brief pause
  }*/

  #ifdef GUI
    establishContact();
    sendData();
  #else
    Serial.println("Setup Complete");  //done trying to get all joints into closed loop control
    Serial.println(optionString);  //print available options for user to select
  #endif
}

void loop() {  //main control loop
  // put your main code here, to run repeatedly:
  #ifdef GUI
    if(millis() - contactTimer > 10000) {  //if > 10sec since last contact check
      establishContact();  //check connection
    } else {
      sendData();
    }
  #endif

  if(ledState == HIGH) {
    digitalWrite(led, LOW);
    ledState = LOW;
  } else {
    digitalWrite(led, HIGH);
    ledState = HIGH;
  }
  
  if(Serial.available()) {  //if a command has been sent via serial
    Serial.println("incoming");  //alert user
    readInput();  //process command
    return;  //skip the rest of the main loop and start over
  }
  
  if(programRunning == true && isError == false) {  //has to both be program running and no errors
    #ifdef GO
      Serial.println("programRunning == true && !isError");  //helpful to see program progression
    #endif

    for(int i = 0;i < numJoints;i++) {  //loop through array of joints
      moveJoint(allJoints[i]);  //move each joint to next position
      
      for(int j = 0;j < numJoints;j++) {  //loop through array of joints
        if(allJoints[j]->readyToMove == true) {  //if joint is at target angle
          allJointsReady = true;  //flag for program progression
        } else {  //first joint that's not at target ruins it for everyone
          allJointsReady = false;  //at least 1 joint needs more time to get to target position
          break;  //exit the loop. No reason to check other joints
        }  //end if else
      }  //end loop

      if(allJointsReady == true) {  //check if all joints are at target position
        for(int j = 0;j < numJoints;j++) {  //loop through array of joints
          allJoints[j]->readyToMove = false;  //reset flag
          allJoints[j]->targetAngle++;  //increase joint angles array index
          if(allJoints[j]->targetAngle >= 50) {  //if the index is higher than length of array
            allJoints[j]->targetAngle = 0;  //reset index to beginning of array
          }  //end if
        }  //end loop
      }  //end if
    }  //end loop
  }// else {  //if program is not running or there is an error
    //programRunning = false;  //stop program
  //}  //end if else
  /*
  if(isError == false) {  //if there are no errors reported,
    checkForErrors();  //check for some
  }*/
}

void establishContact() {  //check for serial connection with gui, handshake
  while (Serial.available() <= 0) {
    Serial.println("A");
    delay(300);
  }
  contactTimer = millis();  //update timer to time at current connection
}

void sendData() {  //send bool, pos, vel, current updates
  dataToSend = '#';
  if(allJointsHomed) {  //true
    dataToSend += "true";  //homed
  } else {
    dataToSend += "false";
  }
  dataToSend += ",";
  if(programRunning) {  //false
    dataToSend += "true";  //programRunning
  } else {
    dataToSend += "false";
  }
  dataToSend += ",";
  if(isError) {  //false
    dataToSend += "true";  //is error
  } else {
    dataToSend += "false";
  }
  dataToSend += ",";
  if(allJointsReady) {  //false
    dataToSend += "true";  //allJointsReady
  } else {
    dataToSend += "false";
  }
  dataToSend += ",";
  for(int i = 0;i < 6;i++) {
    if(programRunning) {
      dataToSend += allJoints[i]->GetPosition();
      dataToSend += ",";
    } else {
      dataToSend += allJoints[i]->posEstimate;
      dataToSend += ",";
    }
  }
  for(int i = 0;i < 6;i++) {
    dataToSend += allJoints[i]->GetIqMeasured();
    dataToSend += ",";
  }
  for(int i = 0;i < 6;i++) {
    dataToSend += allJoints[i]->GetVelocity();
    dataToSend += ",";
  }
  //dataToSend
  dataToSend += '#';
  Serial.println(dataToSend);
}

void moveJoint(Joint *joint) {  //move joint to target position
  if(joint->readyToMove == false) {  //if joint has yet to reach target position
    joint->posEstimate = joint->GetPosition();  //motor position / gear ratio. Data stored in posEstimate
    float absDiff = (joint->posEstimate - joint->GetTargetPosition());  //difference between current position and target position in motor revs
    #ifdef GO  //for debugging
      Serial << "Pos est: " << joint->posEstimate << ", Target: " << joint->targetAngle <<
             "(" << joint->GetTargetPosition() << ")" << '\n';  //print current and target positions
      Serial << "Velocity: " << joint->GetVelocity() << "Pos diff: " << fabs(absDiff) << '\n';  //print joint output velocity and position difference
      Serial << "Target velocity: " << joint->targetVelocity << '\n';
    #endif
    /*
    if(joint->targetAngle == 0) {
      joint->targetVelocity = (joint->angles[49] - joint->angles[0]) / segTime / 360.0 * 120;
    } else {
      joint->targetVelocity = (joint->angles[joint->targetAngle] - joint->angles[joint->targetAngle - 1]) / segTime / 360.0 * 120;
    }
    joint->SetLimits(fabs(joint->targetVelocity), motorCurrentLimit);*/
    joint->SetPosition(joint->GetTargetPosition());  //command motor to target position in motor revs
    
    if(fabs(absDiff) <= posDiffThreshold) {  //if motor is close enough to target position
      joint->readyToMove = true;  //this joint is ready to move to next position
      #ifdef DEBUG
        Serial << "Target position reached" << '\n';
      #endif
    }  //end if
  }  //end if
}

void readInput() {  //proccess data input from user
  int index = 0;  //index to read all incoming data bytes and store in array
  char inByte;  //current byte from user
  bool incomingData = false;  //is there data to proccess
  char startByte = '<';  //marker to signal incoming data
  char endByte = '>';  //marker to signal end of data input
  //bool isNeg = false;
  
  doneReading = false;  //flag that user input has not been read completely

  while(Serial.available() > 0 && doneReading == false) {  //loop through incoming data. Stop when reach end marker
    inByte = Serial.read();  //store byte of data

    if(incomingData) {  //if data has been flagged by the start marker
      if(inByte != endByte) {  //if data is not completely read
        inBytes[index] = inByte;  //store byte in array
        index++;  //advance array index
      } else {  //if incoming data is finished being read
        inBytes[index] = '\0';  //end array with null
        incomingData = false;  //change flag to say no more incoming data
        index = 0;  //reset array index
        doneReading = true;  //set flag to signal incoming data has been read and stored in array
      }
    } else if(inByte == startByte) {  //if the incoming byte is the start marker
      incomingData = true;  //flag as user input
    }  //end if else
  }  //end loop

  if(doneReading) {  //if user input has been read completely
    #ifdef DEBUG  //for debugging
      Serial << "inByte: " << inBytes[0] << '\n';  //print first byte in user input array
    #endif
    if(inBytes[0] == 'x') {  //attempt to clear errors
      isError = false;  //reset error flag
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        allJoints[i]->ClearErrors();  //clear joint errors
        checkJointErrors(allJoints[i]);  //check to see if errors cleared successfully
      }  //end loop
      
      if(isError == true) {  //if theres still an error
        Serial << "Problems persist" << '\n';  //youre screwed i guess
      } else {  //if all errors were cleared
        Serial << "All errors cleared" << '\n';  //celebrate
      }  //end if else
    } else if(inBytes[0] == 'c') {  //calibration sequence
      Joint *joint = allJoints[inBytes[1]];  //pointer to joint to calibrate
      
      joint->calibrated = false;  //reset calibration flag
      Serial << "Calibrating " << jointNames[inBytes[1]] << " joint..." << '\n';  //display which joint is being calibrated
      joint->Calibrate();  //attempt to calibrate joint

      while(joint->GetCurrentState() != 1) {  //1 is idle
        //wait for calibration to finish
      }  //end loop
      
      checkJointErrors(joint);  //check if calibration was successful
      if(isError == true) {  //if theres an error
        Serial.println(optionString);  //print option string again and wait for user input
        return;  //give up. exit function
      } else {  //if theres no error
        joint->calibrated = true;  //flag joint as calibrated
        Serial << "Calibration successful" << '\n';  //alert user of successful calibration
      }  //end if else
    } else if(inBytes[0] == 's') {  //stop everything
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        allJoints[i]->StopMotion();  //stop joint motion
      }  //end loop
      
      Serial.println("Movement stopped");  //alert user motion has stopped
      programRunning = false;  //stop program from advancing
    } else if(inBytes[0] == 'r') {  //run program
      Serial << "Run program selected" << '\n';
      bool problem = false;  //is there a reason to not run program
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        if(allJoints[i]->calibrated == false) {  //check if joint is calibrated. If not
          Serial << jointNames[i] << " not calibrated. Calibrate joint first" << '\n';  //alert user to calibrate joint
          problem = true;  //flag theres a problem
        }  //end if
      }  //end loop
      Serial << "All active joints calibrated" << '\n';
      
      if(problem == true) {  //if theres a problem
        Serial.println(optionString);  //print options and wait for user input
        return;  //exit function
      }  //end if

      //check control_mode == position control
      
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        if(allJoints[i]->homed == false) {  //check if joint has been homed. If not
          Serial << "Not all joints are homed. Homing joints..." << '\n';  //alert user
          homeJoints();  //home all joints
          if(isError == true) {  //if there was a problem
            Serial.println(optionString);  //print options and wait for user input
            return;  //exit function
          } else {  //if there was no problem
            break;  //exit loop. joints homed successfully so no reason to check them again
          }  //end if else
        }  //end if
      }  //end loop
      Serial << "All active joints homed" << '\n';
      
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        if(allJoints[i]->atStart == false) {  //check if joint is at start position. If not
          Serial.println("Moving joints to start position...");  //alert user
          toStartPosition();  //attempt to move joints to start position
          if(isError == true) {  //if theres a problem
            Serial.println(optionString);  //print options and wait for user input
            return;  //exit function
          } else {  //if theres no problem
            break;  //exit loop. all joints at start position. No need to check the rest
          }  //end if else
        }  //end if
      }  //end loop
      Serial << "All active joints at start position" << '\n';
      
      for(int i = 0;i < numJoints;i++) {  //loop through array of joints
        if(allJoints[i]->GetCurrentState() != 8) {  //if joint is not already in closed loop control
          allJoints[i]->ClosedLoopCtrl();  //attempt to put it into closed loop control
          delay(stateChangeTime);  //wait 1 odrive control loop
          if(allJoints[i]->GetCurrentState() != 8) {  //check if successful. if not
            Serial << jointNames[i] << " failed to enter closed loop control" << '\n';  //alert user
            checkJointErrors(allJoints[i]);  //check joint for errors
          }  //end if
        }  //end if
      }  //end loop
      Serial << "All active joints in closed loop control" << '\n';
      
      if(isError == false) {  //if theres no problem
        Serial.println("Starting program");  //alert user to start of program
          delay(500);  //pause for effect
          programRunning = true;  //flag program to advance
      } else {  //if theres a problem
        Serial.println(optionString);  //print option string again and wait for user input
        return;  //exit function
      }  //end if else
      Serial << "Running program..." << '\n';
    } else if(inBytes[0] == 'h') {  //home joints
      Serial.println("Homing joints...");  //alert user
      homeJoints();  //attempt to home joints
      
      if(isError == true) {  //if theres a problem
        Serial << "Error occurred" << '\n';
        Serial.println(optionString);  //print options and wait for user input
        return;  //exit function
      }  //end if
    } else if(inBytes[0] == 'p') {  //move to start position
      Serial.println("Moving to start position...");  //alert user
      toStartPosition();  //do the thing

      if(isError == true) {  //if theres a problem
        Serial << "Error occurred" << '\n';
        Serial.println(optionString);  //print options and wait for user input
        return;  //exit function
      }  //end if
    }  //end if else
    doneReading = false;  //reset flag
    Serial.println(optionString);  //print option string again and wait for user input
  }  //end if(doneReading)
}

void checkForErrors() {  //helper function to check all joints for errors
  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    Serial.println("joint errkrs");
    checkJointErrors(allJoints[i]);  //check each joint for errors
    Serial.println("jug");
    #ifdef GUI
      if(isError) {  //if an error was found
        Serial << '<' << errorString << i << '>' << '\n';  //send errors + joint index wrapped in start/end chars
      }
    #endif
  }  //end loop
  #ifdef GUI
    errorString = "< , , , ,6>";  //signal end of joint errors
    Serial << errorString << '\n';  //send it
  #endif
}

void checkJointErrors(Joint *joint) {  //checks joint for motor, encoder and axis errors and prints them
  Serial.println("gud");
  //joint->GetErrors(errorVals);  //get errors from odrive, store in array
    Serial.println("got errors");
  if(errorVals[0] != 0 || errorVals[1] != 0 || errorVals[2] != 0) {  //if any error value is nonzero, theres an error
    programRunning = false;  //change flag to stop program from running
    isError = true;  //flag the presence of an error
    #ifdef GUI
      errorString = jointNames[joint->node_id];  //print which joint has an error
      errorString += ":,";
    #else
      Serial << jointNames[joint->node_id] << ":" << '\n';  //print which joint has an error
    #endif
  }  //end if
  
  if(errorVals[0] != 0) {  //if theres a motor error
    #ifdef GUI
      errorString += "Motor error: ";  //print
      errorString += int(errorVals[0]);  //display error
      errorString +=",";
    #else
      Serial << '\t' << "Motor error: ";  //print
      Serial.println(motorError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
  
  if(errorVals[1] != 0) {  //if theres an encoder error
    #ifdef GUI
      errorString += "Encoder error: ";  //print
      errorString += int(errorVals[1]);  //display error
      errorString += ",";
    #else
      Serial << '\t' << "Encoder error: ";  //print
      Serial.println(encoderError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
  
  if(errorVals[2] !=0) {  //if theres an axis error
    #ifdef GUI
      errorString += "Axis error: ";  //print
      errorString += int(errorVals[2]);  //display error
      errorString += ",";
    #else
      Serial << '\t' << "Axis error: ";  //print
      Serial.println(axisError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
}

void homeJoints() {  //move all joints to home position. This will straighten the legs
  if(!isError) {  //if there are no errors
    checkForErrors();  //check for some
    if(isError) {  //if an error is found
      return;  //exit function
    }  //end if
  } else {  //if there is an error
    #ifdef DEBUG
      Serial << "Error" << '\n';
      checkForErrors();
    #endif
    return;  //exit function, dont move
  }  //end if else

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    allJoints[i]->SetControlMode(2);  //set control mode to velocity control
    if(allJoints[i]->GetCurrentState() != 8) {  //8 is closed loop control
      allJoints[i]->ClosedLoopCtrl();  //attempt to put joint into closed loop control
      delay(stateChangeTime);  //wait 1 odrive control loop for joint to change state
      if(allJoints[i]->GetCurrentState() != 8) {  //check if joint made it into closed loop control
        checkJointErrors(allJoints[i]);  //if not, why?
      }  //end if
    }  //end if
  }  //end loop
  
  if(isError == true) {  //if there is an error
    return;  //exit function, dont move
  }  //end if

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    if(allJoints[i]->homed == false) {  //if this joint is not at its home position
      while(allJoints[i]->homed == false) {
        if(Serial.available()) {  //check for new user input
          readInput();  //if there is new input, deal with it
          return;  //exit function
        }  //end if
        
        if(fabs(allJoints[i]->GetPotVal() - homePotVal) < posDiffThreshold) {  //if close enough
          allJoints[i]->SetVelocity(0);  //stop motion
          allJoints[i]->homed = true;  //joint is at its home position
          allJoints[i]->SetLinearCount(0);  //set encoder position == 0
          allJoints[i]->SetPosition(0);  //set target position == 0
          allJoints[i]->SetControlMode(3);  //set control mode to position control
        } else if(allJoints[i]->GetPotVal() < homePotVal) {  //if not close enough
          allJoints[i]->SetVelocity(1);  //command joint to turn at 1rev/s
        } else if(allJoints[i]->GetPotVal() > homePotVal) {  //if not close enough the other direction
          allJoints[i]->SetVelocity(-1);  //command joint to turn at 1rev/s the other direction
        }  //end if else
      }  //end while
      
      if(!isError) {  //if there are no errors
        checkForErrors();  //check for some
        if(isError) {  //if an error is found
          #ifdef GUI
            //do nothing
          #else
            Serial << "Homing sequence failed" << '\n';  //alert user
          #endif
          return;  //exit function
        }  //end if isError
      }  //end if isError
    }  //end if allJoints[i]->homed
  }  //end loop

  allJointsHomed = true;
  #ifdef GUI
    //do nothing
  #else
    Serial.println("Homing sequence completed");  //if you made it this far, all joints are in position
  #endif
}

void toStartPosition() {  //move all joints to start position
  if(!isError) {  //if there is no error
    checkForErrors();  //check for some
    if(isError) {  //if an error is found
      return;  //exit function
    }  //end if
  } else {  //if there is an error
    return;  //exit function, dont move
  }  //end if else

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    allJoints[i]->SetLimits(2.0, 25);  //set a modest velocity. Slow and steady but not low enough to error out
    if(allJoints[i]->GetCurrentState() != 8) {  //8 is closed loop control
      allJoints[i]->ClosedLoopCtrl();  //attemp to put joint into closed loop control
      delay(stateChangeTime);  //wait 1 odrive control loop to set state
      if(allJoints[i]->GetCurrentState() != 8) {  //if joint is still not in closed loop control
        Serial << jointNames[i] << " failed to enter closed loop control" << '\n';  //alert user
        checkJointErrors(allJoints[i]);  //check for errors, print them
      }  //end if currentState != 8
    }  //end if
  }  //end loop


  if(isError == true) {  //if theres an error
    return;  //exit function, dont move
  }

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    if(allJoints[i]->atStart == false) {  //if joint is not yet at start position
      if(Serial.available()) {  //check for new user command
        readInput();  //if theres user input, deal with it
        return;  //exit function
      }  //end if

      if(!isError) {  //if there are no errors
        checkForErrors();  //check for some
        if(isError) {  //if an error is found
          return;  //exit function
        }  //end if
      } else {  //if there is an error
        #ifdef GUI
          //do nothing
        #else
          Serial << "Failed to move joints to start position" << '\n';  //alert user
        #endif
        return;  //exit function
      }  //end if else
    
      if(fabs(allJoints[i]->GetPosition() - allJoints[i]->GetTargetPosition()) <= posDiffThreshold) {  //if joint is close enough to target position
        allJoints[i]->atStart = true;  //joint is ready to move to next position
        allJoints[i]->readyToMove = true;  //at target/start position, ready to move to next position
        allJoints[i]->targetAngle++;
      } else {  //if joint is not close enough to target position
        allJoints[i]->SetPosition(allJoints[i]->GetTargetPosition());  //command joint to move to target position
      }  //end if else
    }  //end if
  }  //end for loop

  #ifdef GUI
    //do nothing
  #else
    Serial.println("At start position");  //if you made it this far, joints are all ready to go
  #endif
}
