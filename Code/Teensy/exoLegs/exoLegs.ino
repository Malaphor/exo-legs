#include <cube_spline.h>
#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "Joint.h"


// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//read only and #define
#define DEBUG
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
const int rightHipOffset = 0;  //pot voltage when hip is at 0deg
const int rightKneeOffset = 1.71;  //pot voltage when knee is at 0deg
const int rightAnkleOffset = 459;  //pot voltage when ankle is at 0deg
const int leftHipOffset = 0;  //pot voltage when hip is at 0deg
const int leftKneeOffset = 989;  //pot voltage when knee is at 0deg
const int leftAnkleOffset = 459;  //pot voltage when ankle is at 0deg
const float posDiffThreshold = 0.01;  //0.02how close to target voltage before its ready, in volts or revs
const float motorCurrentLimit = 15.0;  //max motor current
const int numJoints = 1;  //number of active joints, used for testing

const float stateChangeTime = 1.0 / 799.0;  //time to wait for odrive axis to change states
int led = 13;  //onboard led pin
int ledState = LOW;  //LOW = off. Keep track of LED state

//program advancement variables
bool programRunning = false;  //is program running?
int leftLegTarget = 0;  //start array index for left leg joints -------------------- CAHNGE BACK TO 24 -----------------
int rightLegTarget = 0;  //start array index for right leg joints
bool allJointsReady = false;  //are all joints at target angle?
bool allJointsHomed = false;  //homing sequence completed for all joints
unsigned long sendCmdInterval = 0;

//serial reading
char inBytes[32];  //array to read in commands via serial, could probably be smaller
bool doneReading = false;  //finished reading cmd via serial?
float inVal = 0;  //manual pos cmd via serial. Not implemented
String optionString = "Options: <x> to clear errors, <c1> to calibrate axis1, <h> to home joints, <p> to start position, <r> to run program, <s> to stop";

//data transfer
String dataToSend = "";  //string of data values to send to gui
String errorString = "";  //string of all current errors to send to gui
unsigned long contactTimer = 0;

//speed & weight variables
float segTime = 0.08;  //0.0314;  //0.0314 time in sec between angle datapoints, affects walk speed
float weight = 53.0;  //body weight in kg, measured by current draw
float bodyWeight = 50.0;  //% body weight supported by lift

//error variables
uint64_t errorVals[3] = {0, 0, 0};  //initialize errors to 0, no errors
uint32_t axisError = 0;  //32bit axis error
uint64_t motorError = 0;  //64bit motor error
uint32_t encoderError = 0;  //32bit encoder error
uint32_t currentState = 1;  //1 == IDLE
bool isError = false;  //is there currently an error?

unsigned long canMsgTimeout = 0;  //used to wait for CAN message response

//cubic spline interpolation
S output;  //struct
S *output_ptr = NULL;  //pointer to struct
int num_angles = 52;  //needed for cubic spline input
float csVal = 0;  //val to be interpolated in sec after start of gait
float csResult = 0;  //output of interpolater, arg of joint->SetPosition()
int csError = 0;  //checks if value is in domain of dataset
unsigned long startGaitTime = 0;  //time when gait starts in millis()
float endGaitTime = segTime * (num_angles - 1) - 0.01;  //total time for gait in sec
float currentGaitTime = 0;  //time after start of gait in sec
bool restartGait = true;  //if reached end of gait, restart
float csXVals[52] = {};  //segTime vals corresponding to joint angles

//positive angle is flexion, negative is extension
float hipAngles[52] = {19.33, 18.92, 18.45, 17.94, 17.3, 16.4, 15.18, 13.67, 11.97, 10.21, 8.48, 6.74, 4.94, 3.13, 1.42,
                     -0.13, -1.54, -2.87, -4.12, -5.3, -6.4, -7.43, -8.39, -9.27, -10.02, -10.61, -10.95, -10.91, -10.31,
                     -9.0, -6.95, -4.25, -1.05, 2.42, 5.93, 9.22, 12.11, 14.55, 16.53, 18.13, 19.45, 20.54, 21.38, 21.84,
                     21.87, 21.5, 20.84, 20.09, 19.5, 19.18, 19.01, 19.5
                    };
float kneeAngles[52] = {3.97, 7.0, 10.52, 14.12, 17.38, 19.84, 21.27, 21.67, 21.22, 20.20, 18.86, 17.35, 15.73, 14.08, 12.5,
                      11.09, 9.91, 8.97, 8.28, 7.86, 7.72, 7.94, 8.6, 9.76, 11.5, 13.86, 16.97, 20.96, 26.0, 32.03, 38.74,
                      45.6, 52.05, 57.54, 61.66, 64.12, 64.86, 63.95, 61.59, 57.97, 53.27, 47.58, 40.94, 33.46, 25.38,
                      17.27, 9.94, 4.31, 1.12, 0.54, 2.21, 4
                     };
float ankleAngles[51] = {0.02, -2.06, -3.88, -4.6, -3.98, -2.4, -0.45, 1.45, 3.04, 4.27, 5.13, 5.71, 6.1, 6.43, 6.76, 7.12,
                       7.54, 7.99, 8.44, 8.86, 9.23, 9.51, 9.62, 9.43, 8.7, 7.2, 4.69, 1.15, -3.26, -8.17, -13.05, -17.13,
                       -19.52, -19.77, -18.12, -15.29, -12.04, -8.85, -5.96, -3.51, -1.64, -0.5, -0.07, -0.16, -0.42, -0.52,
                       -0.26, 0.36, 1.0, 1.2, 0.58
                      };
float *hipAnglesPtr = hipAngles;  //pointer to float array
float *kneeAnglesPtr = kneeAngles;  //pointer to float array
float *ankleAnglesPtr = ankleAngles;  //pointer to float array


ODriveTeensyCAN odriveCAN(1000000);  //CAN bus object initialized with speed of 1Mbps
ODriveTeensyCAN *odriveCANPtr = &odriveCAN;  //pointer to CAN bus object
CAN_message_t inMsg;  //incoming CAN message
HeartbeatMsg_t returnVals;  //struct that holds heartbeat message data
EncoderEstimatesMsg_t encoderEstimates;  //struct that holds encoder data
IqMsg_t iqVals;  //struct that holds current setpoint and measured current

//joint objects ---------------------------change kneeAnglesPtr
Joint leftKnee(0, leftPosPinKnee, leftKneeOffset, hipAnglesPtr, leftLegTarget, odriveCANPtr);  //CAN node_id, potPin, offset, angles array, leg starting target, CANbus
Joint leftAnkle(1, leftPosPinAnkle, leftAnkleOffset, ankleAnglesPtr, leftLegTarget, odriveCANPtr);
Joint rightKnee(2, rightPosPinKnee, rightKneeOffset, kneeAnglesPtr, rightLegTarget, odriveCANPtr);
Joint rightAnkle(3, rightPosPinAnkle, rightAnkleOffset, ankleAnglesPtr, rightLegTarget, odriveCANPtr);
Joint leftHip(4, leftPosPinHip, leftHipOffset, hipAnglesPtr, leftLegTarget, odriveCANPtr);
Joint rightHip(5, rightPosPinHip, rightHipOffset, hipAnglesPtr, rightLegTarget, odriveCANPtr);

//pointers to joint objects
Joint *leftKneeR = &leftKnee;
Joint *leftAnkleR = &leftAnkle;
Joint *rightKneeR = &rightKnee;
Joint *rightAnkleR = &rightAnkle;
Joint *leftHipR = &leftHip;
Joint *rightHipR = &rightHip;

Joint *allJoints[] = {leftKneeR, leftAnkleR, rightKneeR, rightAnkleR, leftHipR, rightHipR};  //array of pointers to joint objects for looping

//names corresponding to joint objects. Must be in the same order as allJoints[]. For error reporting/debug
const char *jointNames[] = {"Left knee", "Left ankle", "Right knee", "Right ankle", "Left hip", "Right hip"};  


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //start Serial monitor
  pinMode(led, OUTPUT);  //led pin as output
  digitalWrite(led, HIGH);  //turn on led
  ledState = HIGH;  //update LED state

  contactTimer = millis();
  while(!Serial) {
    //wait for computer serial connection
    if(millis() - contactTimer > 5000) {
      break;
    }
  }
  delay(100);  //brief pause

  #ifdef DEBUG  //debug var to help see code stages
    Serial.println("Waiting for heartbeat");
    for(int i = 0;i < numJoints;i++) {  //step through array of joints
      //Serial << odriveCAN.Heartbeat() << '\n';  //rewrite to wait for specific heartbeat
    }
  #endif
  
  do {  //do this first
    if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message on the CAN bus
      uint32_t axis_id = inMsg.id >> 6;  //get axis id
      uint8_t cmd_id = inMsg.id & 0x01F;  //get command id
      if(cmd_id == ODriveTeensyCAN::CMD_ID_ODRIVE_HEARTBEAT_MESSAGE) {  //if command id is for heartbeat message
        handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //take care of it
        break;  //exit do while loop
      }  //end if
    }  //end do
  } while(millis() - canMsgTimeout < 2000);  //only loop for 2sec

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

  fillSegTimeArray();  //fill array w/time vals corresponding to angles
  output.x = csXVals;  //initial x values
  output.y = ankleAngles;  //initial y values
  output_ptr = nat_cubic_spline(num_angles, &output);  //performs natural cubic spline of given dataset

  #ifdef GUI  //if using gui
    establishContact();  //handshake with gui comp
    sendData();  //send bool, pos, vel, current updates
  #else  //if using serial monitor
    Serial.println("Setup Complete");  //done trying to get all joints into closed loop control
    Serial.println(optionString);  //print available options for user to select
  #endif

  sendCmdInterval = millis();
}

void loop() {  //main control loop
  // put your main code here, to run repeatedly:
  #ifdef GUI
    if(millis() - contactTimer > 10000) {  //if > 10sec since last contact check
      establishContact();  //check connection
    } else {
      sendData();  //send bool, pos, vel, current updates
    }
  #endif

  if(ledState == HIGH) {  //if led is on
    digitalWrite(led, LOW);  //turn it off
    ledState = LOW;  //update
  } else {  //if led is off
    digitalWrite(led, HIGH);  //turn it on
    ledState = HIGH;  //update
  }  //end if else
  
  if(Serial.available()) {  //if a command has been sent via serial
    //Serial.println("incoming");  //alert user
    readInput();  //process command
    return;  //skip the rest of the main loop and start over
  }

  if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
    #ifdef DEBUGg
      Serial.println("CAN message");
    #endif
    uint32_t axis_id = inMsg.id >> 5;  //get axis id
    /*Serial.print(inMsg.id);
    Serial.print("\t");
    Serial.println(axis_id);*/
    if(axis_id == 0) {
      handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //deal with it
    }
  }  //end if
  
  if(programRunning == true && isError == false) {  //has to both be program running and no errors
    #ifdef DEBUGg
      Serial.println("programRunning == true && !isError");  //helpful to see program progression
      Serial.println(leftKnee.readyToMove, allJointsReady);
    #endif

    if(millis() - sendCmdInterval > 10) {  //run code inside every 10ms (100Hz)
    
    for(int i = 0;i < numJoints;i++) {  //loop through array of joints
      //moveJoint(allJoints[i]);  //move each joint to next position
      
      if(currentGaitTime > endGaitTime) {  //if time after start of gait is greater than total time for gait to finish
        restartGait = true;  //restart gait time at current system time
      }  //end if
    
    if(restartGait == true) {
      startGaitTime = millis();
      restartGait = false;
    }

      currentGaitTime = (millis() - startGaitTime) / 1000.0;  //time after start of gait, converted to sec
      
      csVal = currentGaitTime;  //set value to interpolate

      if((csError = evaluate(output_ptr, csVal, &csResult)) < 0) {  //if theres an error during evaluation
        Serial << "CS Error: " << csError << '\n';  //print the error. -1 csVal too small, -2 csVal too big, -3 it broke
      }  //end if
      
      csResult = csResult * 120 / 360.0;  //convert joint output deg to motor revs w/120:1 gear ratio

      allJoints[i]->SetPosition(csResult);
      /*
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
      }  //end if*/
    }  //end loop
    sendCmdInterval = millis();
    }
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
    dataToSend += allJoints[i]->posEstimate;
    dataToSend += ",";
  }
  for(int i = 0;i < 6;i++) {
    dataToSend += allJoints[i]->iqMeasured;
    dataToSend += ",";
  }
  for(int i = 0;i < 6;i++) {
    dataToSend += allJoints[i]->velEstimate;
    dataToSend += ",";
  }
  //dataToSend
  dataToSend += '#';
  Serial.println(dataToSend);
}

void moveJoint(Joint *joint) {  //move joint to target position
  if(joint->readyToMove == false) {  //if joint has yet to reach target position
    //joint->posEstimate = joint->GetPosition();  //motor position / gear ratio. Data stored in posEstimate
    float absDiff = (joint->posEstimate - joint->GetTargetPosition());  //difference between current position and target position in motor revs
    #ifdef DEBUG  //for debugging
      Serial << "Pos est: " << joint->posEstimate << ", Target: " << joint->targetAngle <<
             "(" << joint->GetTargetPosition() << ")" << '\n';  //print current and target positions
      Serial << "Velocity: " << joint->GetVelocity() << ", Pos diff: " << fabs(absDiff) << '\n';  //print joint output velocity and position difference
      //Serial << "Target velocity: " << joint->targetVelocity << '\n';
    #endif

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
      allJoints[inBytes[1]]->calibrated = false;  //reset calibration flag
      Serial << "Calibrating " << jointNames[inBytes[1]] << " joint..." << '\n';  //display which joint is being calibrated
      allJoints[inBytes[1]]->Calibrate();  //attempt to calibrate joint

      while(allJoints[inBytes[1]]->currentState != 1) {  //1 is idle. State is also IDLE if theres an error
        //wait for calibration to finish
        if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
          uint32_t axis_id = inMsg.id >> 6;  //get axis id
          if(axis_id == inBytes[1]) {  //if axis id is same as currently calibrating axis
            handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //deal with it
          }  //end if
        }  //end if
      }  //end loop
      
      if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
        uint32_t axis_id = inMsg.id >> 6;  //get axis id
        if(axis_id == inBytes[1]) {  //if axis id is same as currently calibrating axis
          handleCANMsg(axis_id, inMsg, allJoints[inBytes[1]]);  //deal with it
        }  //end if
      }  //end if
      
      checkJointErrors(allJoints[inBytes[1]]);  //check if calibration was successful
      if(isError == true) {  //if theres an error
        Serial.println("Calibration unsuccessful");  //notify user
        Serial.println(optionString);  //print option string again and wait for user input
        return;  //give up. exit function
      } else {  //if theres no error
        allJoints[inBytes[1]]->calibrated = true;  //flag joint as calibrated
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
        if(allJoints[i]->currentState != 8) {  //if joint is not already in closed loop control
          allJoints[i]->ClosedLoopCtrl();  //attempt to put it into closed loop control
          delay(stateChangeTime);  //wait 1 odrive control loop
          if(allJoints[i]->currentState != 8) {  //check if successful. if not
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
    if(allJoints[i]->isError) {  //if joint has an error
      checkJointErrors(allJoints[i]);  //check joint for errors
      #ifdef GUI
        Serial << '<' << errorString << i << '>' << '\n';  //send errors + joint index wrapped in start/end chars
      #endif
    }  //end if
  }  //end loop
  #ifdef GUI
    errorString = "< , , , ,6>";  //signal end of joint errors
    Serial << errorString << '\n';  //send it
  #endif
}

void checkJointErrors(Joint *joint) {  //checks joint for motor, encoder and axis errors and prints them

  odriveCANPtr->GetMotorError(joint->node_id);  //send get motor error message
  do {  //do this first
    if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message on the CAN bus
      uint32_t axis_id = inMsg.id >> 6;  //get axis id
      uint8_t cmd_id = inMsg.id & 0x01F;  //get command id
      if(cmd_id == ODriveTeensyCAN::CMD_ID_GET_MOTOR_ERROR && axis_id == joint->node_id) {  //if command id is for motor error message
        handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //take care of it
        break;  //exit do while loop
      }  //end if
    }  //end do
  } while(millis() - canMsgTimeout < 500);  //only loop for 1/2sec
  
  odriveCANPtr->GetEncoderError(joint->node_id);  //send get encoder error message
  do {  //do this first
    if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message on the CAN bus
      uint32_t axis_id = inMsg.id >> 6;  //get axis id
      uint8_t cmd_id = inMsg.id & 0x01F;  //get command id
      if(cmd_id == ODriveTeensyCAN::CMD_ID_GET_ENCODER_ERROR && axis_id == joint->node_id) {  //if command id is for motor error message
        handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //take care of it
        break;  //exit do while loop
      }  //end if
    }  //end do
  } while(millis() - canMsgTimeout < 500);  //only loop for 1/2sec
  
  odriveCANPtr->GetControllerError(joint->node_id);  //send get controller error message
  do {  //do this first
    if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message on the CAN bus
      uint32_t axis_id = inMsg.id >> 6;  //get axis id
      uint8_t cmd_id = inMsg.id & 0x01F;  //get command id
      if(cmd_id == ODriveTeensyCAN::CMD_ID_GET_CONTROLLER_ERROR && axis_id == joint->node_id) {  //if command id is for motor error message
        handleCANMsg(axis_id, inMsg, allJoints[axis_id]);  //take care of it
        break;  //exit do while loop
      }  //end if
    }  //end do
  } while(millis() - canMsgTimeout < 500);  //only loop for 1/2sec
  
  #ifdef GUI
    errorString = jointNames[joint->node_id];  //print which joint has an error
    errorString += ":,";
  #else
    Serial << jointNames[joint->node_id] << ":" << '\n';  //print which joint has an error
  #endif
  
  if(joint->motorError != 0) {  //if theres a motor error
    #ifdef GUI
      errorString += "Motor error: ";  //print
      errorString += int(joint->motorError);  //display error
      errorString +=",";
    #else
      Serial << '\t' << "Motor error: ";  //print
      Serial.println(joint->motorError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
  
  if(joint->encoderError != 0) {  //if theres an encoder error
    #ifdef GUI
      errorString += "Encoder error: ";  //print
      errorString += int(joint->encoderError);  //display error
      errorString += ",";
    #else
      Serial << '\t' << "Encoder error: ";  //print
      Serial.println(joint->encoderError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
  
  if(joint->axisError !=0) {  //if theres an axis error
    #ifdef GUI
      errorString += "Axis error: ";  //print
      errorString += int(joint->axisError);  //display error
      errorString += ",";
    #else
      Serial << '\t' << "Axis error: ";  //print
      Serial.println(joint->axisError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
  
  if(joint->controllerError !=0) {  //if theres an axis error
    #ifdef GUI
      errorString += "Controller error: ";  //print
      errorString += int(joint->controllerError);  //display error
      errorString += ",";
    #else
      Serial << '\t' << "Controller error: ";  //print
      Serial.println(joint->controllerError, HEX);  //display error in hexadecimal format
    #endif
  }  //end if
}

void homeJoints() {  //move all joints to home position. This will straighten the legs
  if(isError == true) {  //if there is an error
    #ifdef DEBUG
      Serial << "Error" << '\n';
      checkForErrors();
    #endif
    return;  //exit function, dont move
  }  //end if else

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    allJoints[i]->SetControlMode(ODriveTeensyCAN::VELOCITY_CONTROL);  //set control mode to velocity control
    if(allJoints[i]->currentState != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) {  //8 is closed loop control
      allJoints[i]->ClosedLoopCtrl();  //attempt to put joint into closed loop control
      delay(stateChangeTime);  //wait 1 odrive control loop for joint to change state
      if(allJoints[i]->currentState != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) {  //check if joint made it into closed loop control
        checkJointErrors(allJoints[i]);  //if not, why?
      }  //end if
    }  //end if
  }  //end loop
  
  if(isError == true) {  //if there is an error
    return;  //exit function, dont move
  }  //end if

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    if(allJoints[i]->homed == false) {  //if this joint is not at its home position
      while(allJoints[i]->homed == false && !isError) {  //while joint is not homed and there are no errors
        allJoints[i]->GetPotVal();  //query pot voltage from odrive
        if(Serial.available()) {  //check for new user input
          readInput();  //if there is new input, deal with it
          return;  //exit function
        }  //end if
        
        if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
          uint32_t axis_id = inMsg.id >> 6;  //get axis id
          if(axis_id == allJoints[i]->node_id) {  //if axis id is same as currently homing axis
            handleCANMsg(axis_id, inMsg, allJoints[i]);  //deal with it
          }  //end if
        }  //end if
        
        if(fabs(allJoints[i]->potVal - homePotVal) < posDiffThreshold) {  //if close enough
          allJoints[i]->SetVelocity(0);  //stop motion
          allJoints[i]->homed = true;  //joint is at its home position
          allJoints[i]->SetLinearCount(0);  //set encoder position == 0
          allJoints[i]->SetPosition(0);  //set target position == 0
          allJoints[i]->SetControlMode(ODriveTeensyCAN::POSITION_CONTROL);  //set control mode to position control
        } else if(allJoints[i]->potVal < homePotVal) {  //if not close enough
          allJoints[i]->SetVelocity(1);  //command joint to turn at 1rev/s
        } else if(allJoints[i]->potVal > homePotVal) {  //if not close enough the other direction
          allJoints[i]->SetVelocity(-1);  //command joint to turn at 1rev/s the other direction
        }  //end if else
      }  //end while
      
      if(isError) {  //if an error is found
        checkForErrors();  //check for some
        #ifdef GUI
          //do nothing
        #else
          Serial << "Homing sequence failed" << '\n';  //alert user
        #endif
        return;  //exit function
      }  //end if isError
    }  //end if allJoints[i]->homed
  }  //end for loop

  allJointsHomed = true;
  #ifdef GUI
    //do nothing
  #else
    Serial.println("Homing sequence completed");  //if you made it this far, all joints are in position
  #endif
}

void toStartPosition() {  //move all joints to start position
  if(isError) {  //if an error is found
    return;  //exit function, dont move
  }  //end if

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    allJoints[i]->SetLimits(2.0, 25);  //set a modest velocity. Slow and steady but not low enough to error out
    if(allJoints[i]->currentState != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) {  //8 is closed loop control
      allJoints[i]->ClosedLoopCtrl();  //attemp to put joint into closed loop control
      delay(stateChangeTime);  //wait 1 odrive control loop to set state
      if(allJoints[i]->currentState != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) {  //if joint is still not in closed loop control
        Serial << jointNames[i] << " failed to enter closed loop control" << '\n';  //alert user
        checkJointErrors(allJoints[i]);  //check for errors, print them
        if(isError == true) {  //if theres an error
          return;  //exit function, dont move
        }
      }  //end if currentState != 8
    }  //end if
    
    if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
      uint32_t axis_id = inMsg.id >> 6;  //get axis id
      handleCANMsg(axis_id, inMsg, allJoints[i]);  //deal with it
    }  //end if
  }  //end loop

  for(int i = 0;i < numJoints;i++) {  //loop through array of joints
    if(allJoints[i]->atStart == false) {  //if joint is not yet at start position
      if(Serial.available()) {  //check for new user command
        readInput();  //if theres user input, deal with it
        return;  //exit function
      }  //end if
      
      if(odriveCAN.ReadMsg(inMsg)) {  //if theres a message to be read
        uint32_t axis_id = inMsg.id >> 6;  //get axis id
        handleCANMsg(axis_id, inMsg, allJoints[i]);  //deal with it
      }  //end if

      if(isError) {  //if an error is found
        #ifdef GUI
          //do nothing
        #else
          Serial << "Failed to move joints to start position" << '\n';  //alert user
        #endif
        return;  //exit function
      }  //end if
    
      if(fabs(allJoints[i]->posEstimate - allJoints[i]->GetTargetPosition()) <= posDiffThreshold) {  //if joint is close enough to target position
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

void handleCANMsg(int axis_id, CAN_message_t inMsg, Joint *joint) {  //store data received from message
    uint8_t cmd_id = inMsg.id & 0x01F;  //get message command id
    
    switch(cmd_id) {  //test command id
      case (ODriveTeensyCAN::CMD_ID_ODRIVE_HEARTBEAT_MESSAGE):  //if heartbeat
        odriveCAN.Heartbeat(returnVals, inMsg);  //store values from inMsg in returnVals struct
        joint->axisError = returnVals.axisError;  //set joint axis error code
        joint->currentState = returnVals.currentState;  //set joint current state
    
        if(returnVals.motorFlag || returnVals.encoderFlag || returnVals.controllerFlag || joint->axisError != 0) {  //if any errors
          joint->isError = true;  //set joint error flag
          programRunning = false;  //dont try to move joints
        }
        break;  //exit switch
      case (ODriveTeensyCAN::CMD_ID_GET_MOTOR_ERROR):  //if getting motor error
        joint->motorError = odriveCAN.GetMotorErrorResponse(inMsg);  //set joint error code
        break;  //exit swi tch
      case (ODriveTeensyCAN::CMD_ID_GET_ENCODER_ERROR):  //if getting encoder error
        joint->encoderError = odriveCAN.GetEncoderErrorResponse(inMsg);  //set joint error code
        break;  //exit switch
      case (ODriveTeensyCAN::CMD_ID_GET_ENCODER_ESTIMATES):  //if getting pos/vel
        //Serial.println("encoder estimates");
        odriveCAN.GetPositionVelocityResponse(encoderEstimates, inMsg);  //store values from inMsg into encoderEstimates struct
        Serial << "input_pos: " << csResult << '\n';
        Serial << "pos est: " << encoderEstimates.posEstimate << '\n';
        //Serial << "vel_estimate: " << encoderEstimates.velEstimate << '\n';
        joint->posEstimate = encoderEstimates.posEstimate;  //set joint position estimate
        joint->velEstimate = encoderEstimates.velEstimate;  //set joint velocity estimate
        break;  //exit switch
      case (ODriveTeensyCAN::CMD_ID_GET_IQ):  //if getting motor current
        odriveCAN.GetIqResponse(iqVals, inMsg);  //store values
        break;  //exit switch
      case (ODriveTeensyCAN::CMD_ID_GET_ADC_VOLTAGE):  //if getting voltage feedback
        joint->potVal = odriveCAN.GetADCVoltageResponse(inMsg);  //set joint pot value
        break;  //exit switch
      default:  //other/unwanted command
        break;  //exit switch
    }  //end switch
}  //end handleCANMsg

void fillSegTimeArray() {
  for(int i = 0;i < num_angles;i++) {
    csXVals[i] = i * segTime;
  }
}
