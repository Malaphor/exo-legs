#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "joint.h"

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//read only and #define
#define DEBUG 1
#define rightPosPinHip A7  //pin to read pot val for hip pos
#define rightPosPinKnee A8  //pin to read pot val for knee pos
#define rightPosPinAnkle A9  //pin to read pot val for ankle pos
#define leftPosPinHip A4  //pin to read pot val for hip pos
#define leftPosPinKnee A5  //pin to read pot val for knee pos
#define leftPosPinAnkle A6  //pin to read pot val for ankle pos

const float potValToDeg = 2.6;  //convert analog pot val to degrees
const int rightHipOffset = 0;  //potVal when hip is at 0deg
const int rightKneeOffset = 533;  //potVal when knee is at 0deg
const int rightAnkleOffset = 459;  //potVal when ankle = 0deg
const int leftHipOffset = 0;  //potVal when hip is at 0deg
const int leftKneeOffset = 533;  //potVal when knee is at 0deg
const int leftAnkleOffset = 459;  //potVal when ankle = 0deg
const float posDiffThreshold = 0.01;  //how close to target angle before its ready, in motor revs
const int numJoints = 6;  //number of active joints

int led = 13;

//program advancement variables
bool readyToMove = false;  //no errors, is everything calibrated and homed?
bool programRunning = false;  //is program running?
int leftLegTarget = 24;
int rightLegTarget = 0;
bool rightLegReady = false;  //all right joints reached target angle?
bool leftLegReady = false;  //all left joints reached target angle?

//joint homing, start position, calibration
bool allJointsHomed = true;  //all joints at 0 degrees?
bool atStart = true;  //all joints at start angle?

//serial reading
char inBytes[32];  //array to read in commands via serial
bool doneReading = false;  //finished reading cmd via serial?
float inVal = 0;  //manual pos cmd via serial

//speed variables
float segTime = 0.314;  //0.0314 time in sec between angle datapoints, walk speed
float rightHipSpeed = 5.0;  //motor rev/s
float rightKneeSpeed = 5.0;
float rightAnkleSpeed = 5.0;
float leftHipSpeed = 5.0;
float leftKneeSpeed = 0.5;
float leftAnkleSpeed = 5.0;

//position variables
/*int hipMin = 100;  //in counts: -120000;
int hipMax = 150;  //in counts: 40000;
int kneeMin = 100;  //in counts: -6667;
int kneeMax = 150;  //in counts: 120000;
int ankleMin = 570;  //in counts: -33333;
int ankleMax = 250;  //in counts: 26666;
*/

//error variables
uint32_t axisError = 0;
uint32_t motorError = 0;
uint32_t encoderError = 0;
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
float *hipAnglesPtr = hipAngles;
float *kneeAnglesPtr = kneeAngles;
float *ankleAnglesPtr = ankleAngles;


ODriveTeensyCAN odriveCAN;

//joint objects
Joint leftKnee(0, leftPosPinKnee, leftKneeOffset, kneeAnglesPtr, 24);  //node_id, potPin, offset, angles, leg target
Joint leftAnkle(1, leftPosPinAnkle, leftAnkleOffset, ankleAnglesPtr, 24);
Joint rightKnee(2, rightPosPinKnee, rightKneeOffset, kneeAnglesPtr, 0);
Joint rightAnkle(3, rightPosPinAnkle, rightAnkleOffset, ankleAnglesPtr, 0);
Joint leftHip(4, leftPosPinHip, leftHipOffset, hipAnglesPtr, 24);
Joint rightHip(5, rightPosPinHip, rightHipOffset, hipAnglesPtr, 0);

//pointers to joint objects
Joint *leftKneeR = &leftKnee;
Joint *leftAnkleR = &leftAnkle;
Joint *rightKneeR = &rightKnee;
Joint *rightAnkleR = &rightAnkle;
Joint *leftHipR = &leftHip;
Joint *rightHipR = &rightHip;

Joint *allJoints[] = {leftKneeR, leftAnkleR, rightKneeR, rightAnkleR, leftHipR, rightHipR};

char *jointNames[] = {"Left knee", "Left ankle", "Right knee", "Right ankle", "Left hip", "Right hip"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  while(!Serial) {
    //wait for computer serial connection
  }
  while(!odriveCAN.available()) {
    //wait for CAN bus connection
  }
  #ifdef DEBUG
  Serial.println("hey1");
  #endif

  for(int i = 0;i < numJoints;i++) {
    odriveCAN.RunState(allJoints[i]->node_id, ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
    int currentState = odriveCAN.GetCurrentState(allJoints[i]->node_id);
    if(currentState == 8) {
      Serial << jointNames[i] << " ready" << '\n';
    } else {
      Serial << jointNames[i] << " failed to enter closed loop control" << '\n';
    }
    delay(100);
  }
  
  Serial.println("Setup Complete");
  Serial.println("Options: <c1> to calibrate axis1, <h> to home joints, <r> to run program, <s> to stop");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!isError) {
    checkForErrors();
  }
  //motorError = odriveCAN.GetMotorError(axes[1]);
  Serial << odriveCAN.GetCurrentState(allJoints[1]) << '\n';
  
  if(Serial.available()) {
    Serial.println("incoming");
    readInput();
    return;
  }
  
  if(programRunning && isError == false) {
  Serial.println("yo");
  //if(allJointsHomed && atStart) {
    //moveLeftKnee();
//    moveLeftAnkle();
    if(leftKneeReady && leftAnkleReady) {
      leftLegTarget++;
      leftKneeReady = false;
      leftAnkleReady = false;
      
      if(leftLegTarget >= 50) {
        leftLegTarget = 0;
      }
    }
  /*} else {
    programRunning = false;
  }*/
  }
}

/*void moveLeftKnee() {
  if(!leftKneeReady) {
    leftKneePosEstimate = odriveCAN.GetPosition(axes[1]) / 120;  //motor position / gear ratio
    float absDiff = (leftKneePosEstimate - (kneeAngles[leftLegTarget] / 360.0)) * 120;
    #ifdef DEBUG
      Serial << "Pos est: " << leftKneePosEstimate * 120 << ", Target: " << leftLegTarget << "(" << kneeAngles[leftLegTarget] / 360 * 120 << ")" << '\n';
      Serial << "Velocity: " << odriveCAN.GetVelocity(axes[1]) << "Pos diff: " << fabs(absDiff) << '\n';
    #endif
    /*if(leftLegTarget == 0) {
      leftKneeSpeed = (kneeAngles[49] - kneeAngles[leftLegTarget]) / 360 / segTime;
    } else {
      leftKneeSpeed = (kneeAngles[leftLegTarget] - kneeAngles[leftLegTarget - 1]) / 360 / segTime;
    }

    odriveCAN.SetPosition(axes[1], kneeAngles[leftLegTarget] / 360.0 * 120);
          
    if(fabs(absDiff) <= posDiffThreshold) {
      leftKneeReady = true;
    }
  }
}*/

void moveJoint(Joint *joint) {
  if(joint->readyToMove == false) {
    joint->posEstimate = odriveCAN.GetPosition(joint->node_id) / 120.0;  //motor position / gear ratio
    float absDiff = (joint->posEstimate - (joint->angles[joint->targetAngle] / 360.0)) * 120;
    #ifdef DEBUG
      Serial << "Pos est: " << joint->posEstimate * 120 << ", Target: " << joint->targetAngle <<
             "(" << joint->angles[joint->targetAngle] / 360.0 * 120 << ")" << '\n';
      Serial << "Velocity: " << odriveCAN.GetVelocity(joint->node_id) << "Pos diff: " << fabs(absDiff) 
             << '\n';
    #endif
    /*if(leftLegTarget == 0) {
      leftAnkleSpeed = (ankleAngles[49] - ankleAngles[leftLegTarget]) / 360 / segTime;
    } else {
      leftAnkleSpeed = (ankleAngles[leftLegTarget] - ankleAngles[leftLegTarget - 1]) / 360 / segTime;
    }*/

    odriveCAN.SetPosition(joint->node_id, joint->angles[joint->targetAngle] / 360.0 * 120);
          
    if(fabs(absDiff) <= posDiffThreshold) {
      joint->readyToMove = true;
    }
  }
}

void readInput() {
  int index = 0;
  char inByte;
  bool incomingData = false;
  char startByte = '<';
  char endByte = '>';
  bool isNeg = false;

  while(Serial.available() > 0 && doneReading == false) {
    inByte = Serial.read();

    if(incomingData) {
      if(inByte != endByte) {
        inBytes[index] = inByte;
        index++;
      } else {
        inBytes[index] = '\0';
        incomingData = false;
        index = 0;
        doneReading = true;
      }
    } else if(inByte == startByte) {
      incomingData = true;
    }
  }

  if(doneReading) {
    Serial << "inByte: " << inBytes[0] << '\n';
    if(inBytes[0] == 'c') {  //calibration sequence
      allJoints[inBytes[1]]->calibrated = false;
      Serial << "Calibrating " << jointNames[inBytes[1]] << " joint..." << '\n';
      
      Serial << jointNames[inBytes[1]] << " motor calibration" << '\n';
      odriveCAN.RunState(allJoints[inBytes[1]]->node_id, ODriveTeensyCAN::AXIS_STATE_MOTOR_CALIBRATION);
      motorError = odriveCAN.GetMotorError(allJoints[inBytes[1]]->node_id);
      if(motorError == 0) {
        Serial << "Motor calibration successful" << '\n';
      } else {
        Serial << "Motor calibration failed. Error code " << motorError << '\n';
        return;
      }

      Serial << jointNames[inBytes[1]] << " encoder calibration" << '\n';
      odriveCAN.RunState(allJoints[inBytes[1]]->node_id, ODriveTeensyCAN::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
      encoderError = odriveCAN.GetEncoderError(allJoints[inBytes[1]]->node_id);
      if(encoderError == 0) {
        Serial << "Encoder calibration successful" << '\n';
      } else {
        Serial << "Encoder calibration failed. Error code " << encoderError << '\n';
        return;
      }
      
      allJoints[inBytes[1]]->calibrated = true;
      doneReading = false;
    } else if(inBytes[0] == 's') {  //stop everything
      for(int i = 0;i < numJoints;i++) {
        odriveCAN.RunState(allJoints[i]->node_id, ODriveTeensyCAN::AXIS_STATE_IDLE);
      }
      Serial.println("Movement stopped");
      programRunning = false;
    } else if(inBytes[0] == 'r') {  //run program
      bool problem = false;
      for(int i = 0;i < numJoints;i++) {
        if(allJoints[i]->calibrated == false) {
          Serial << jointNames[i] << " not calibrated. Calibration joint first" << '\n';
          problem = true;
        }
      }
      if(problem) {
        Serial.println("Options: <c1> to calibrate axis1, <h> to home joints, <r> to run program, <s> to stop");
        return;
      }
      for(int i = 0;i < numJoints;i++) {
        if(allJoints[i]->homed == false) {
          Serial << jointNames[i] << " not homed. Homing joint..." << '\n';
          homeJoint(allJoints[i]);
        }
      }
      if(!atStart) {
        Serial.println("Moving joints to start position...");
        toStartPosition();
      } else {
        Serial.println("Starting program");
        delay(500);
        programRunning = true;
      }
    } else if(inBytes[0] = 'h') {  //home joints
      Serial.println("Homing joints...");
      //homeAllJoints();
    }
    doneReading = false;
  }
}

void checkForErrors() {
  for(int i = 0;i < numJoints;i++) {
    motorError = odriveCAN.GetMotorError(allJoints[i]);
    encoderError = odriveCAN.GetEncoderError(allJoints[i]);
    axisError = odriveCAN.GetAxisError(allJoints[i]);

    if(motorError != 0 || encoderError != 0 || axisError != 0) {
      programRunning = false;
      isError = true;
      Serial << jointNames[allJoints[i]->node_id] << ":" << '\n';
      Serial << '\t' << "Motor error: " << motorError << '\n';
      Serial << '\t' << "Encoder error: " << encoderError << '\n';
      Serial << '\t' << "Axis error: " << axisError << '\n';
    }
  }
}

void homeJoint(Joint *joint) {
  if(!isError) {
    checkForErrors();
  } else {
    return;
  }
  float posDiff = 0;  //difference between current & home pos in revs
  
  for(int i = 0;i < numJoints;i++) {
    odriveCAN.SetVelocityLimit(allJoints[i]->node_id, 6.0);
  }
  
  while(!joint->homed) {
    if(Serial.available()) {
      readInput();
      return;
    }

    posDiff = (joint->GetPotPos() - joint->potOffset) * potValToDeg / 360.0;  //pos diff from deg to revs
    joint->posEstimate = odriveCAN.GetPosition(joint->node_id);

    if(fabs(posDiff) < posDiffThreshold) {  //if close enough
      joint->homed = true;
    } else {
      odriveCAN.SetPosition(joint->node_id, joint->posEstimate - posDiff);
    }
  }
  Serial.println("Homing sequence completed");
}

void toStartPosition() {
  if(!isError) {
    checkForErrors();
  } else {
    return;
  }

  for(int i = 0;i < numJoints;i++) {
    odriveCAN.SetVelocityLimit(allJoints[i]->node_id, 6.0);
  }
  
  while(!atStart) {
    if(Serial.available()) {
      readInput();
      return;
    }

    for(int i = 0;i < numJoints;i++) {
      if(allJoints[i]->atStart == false) {
        allJoints[i]->posEstimate = odriveCAN.GetPosition(allJoints[i]);
    
        if(fabs(allJoints[i]->posEstimate - (allJoints[i]->angles[0] / 360.0)) <= posDiffThreshold) {
          allJoints[i]->atStart = true;
        } else {
          odriveCAN.SetPosition(allJoints[i]->node_id, allJoints[i]->angles[0] / 360.0);
        }
      }
    }

    for(int i = 0;i < numJoints;i++) {
      if(allJoints[i]->atStart == false) {
        //leave atStart == false
      } else {
        atStart = true;
      }
    }
  }
  Serial.println("At start position");
}
