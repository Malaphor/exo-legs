#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include <joint.h>

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


int led = 13;
int axes[6] = {0, 1, 2, 3, 4, 5}; //{LA, LK, LH, RA, RK, RH}

float axis_pos = 1.0;

//program advancement variables
bool readyToMove = false;  //no errors, is everything calibrated and homed?
bool programRunning = false;  //is program running?
int leftLegTarget = 24;
int rightLegTarget = 0;
bool rightLegReady = false;  //all right joints reached target angle?
bool leftLegReady = false;  //all left joints reached target angle?
bool rightHipReady = false;  //hip angle equal to target angle?
bool rightKneeReady = false;  //knee angle equal to target angle?
bool rightAnkleReady = false;  //ankle angle equal to target angle?
bool leftHipReady = false;  //hip angle equal to target angle?
bool leftKneeReady = false;  //knee angle equal to target angle?
bool leftAnkleReady = false;  //ankle angle equal to target angle?

//joint homing, start position, calibration
bool allJointsHomed = true;  //all joints at 0 degrees?
bool atStart = true;  //all joints at start angle?
bool calibrated = true; //axes calibrated?
bool rightHipHomed = false;  //at 0 degrees? via pot reading
bool rightKneeHomed = false;  //at 0 degrees? via pot reading
bool rightAnkleHomed = false;  //at 0 degrees? via pot reading
bool leftHipHomed = false;  //at 0 degrees? via pot reading
bool leftKneeHomed = false;  //at 0 degrees? via pot reading
bool leftAnkleHomed = false;  //at 0 degrees? via pot reading

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
float rightHipPosEstimate = 0.0;  //motor revs from odrive
float rightKneePosEstimate = 0.0;  //motor revs from odrive
float rightAnklePosEstimate = 0.0;  //motor revs from odrive
float leftHipPosEstimate = 0.0;  //motor revs from odrive
float leftKneePosEstimate = 0.0;  //motor revs from odrive
float leftAnklePosEstimate = 0.0;  //motor revs from odrive
double rightHipPotVal;  //read hip position from pot
double rightKneePotVal;  //read knee position from pot
double rightAnklePotVal;  //read ankle position from pot
double leftHipPotVal;  //read hip position from pot
double leftKneePotVal;  //read knee position from pot
double leftAnklePotVal;  //read ankle position from pot

//error variables
uint32_t axisError = 0;
uint32_t motorError = 0;
uint32_t encoderError = 0;
uint32_t currentState = 1;  //1 == IDLE

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



ODriveTeensyCAN odriveCAN;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  while(!Serial) {
    //wait for computer serial connection
  }
  #ifdef DEBUG
  Serial.println("hey1");
  #endif
  //odriveCAN.SetVelocity(axes[1], leftKneeSpeed);
  odriveCAN.RunState(axes[1], ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
  odriveCAN.RunState(axes[0], ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
  
  Serial.println("Setup Complete");
  Serial.println("Options: <c1> to calibrate axis1, <h> to home joints, <r> to run program, <s> to stop");
}

void loop() {
  // put your main code here, to run repeatedly:
  checkForErrors();
  //motorError = odriveCAN.GetMotorError(axes[1]);
  Serial << odriveCAN.GetCurrentState(axes[1]) << '\n';
  /*if(h == true) {
    return;
  }*/
  if(Serial.available()) {
    Serial.println("incoming");
    readInput();
    return;
  }

  /*
  Serial.println("hey2");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
  Serial.println("hey3");
  axis_pos = odriveCAN.GetPosition(axes[1]);
  Serial.println("hey4");
  odriveCAN.SetPosition(axes[1], axis_pos + 0.1);
  Serial.println(axis_pos);*/
  if(programRunning) {
  Serial.println("yo");
  //if(allJointsHomed && atStart) {
    moveLeftKnee();
    moveLeftAnkle();
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

void moveLeftKnee() {
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
    }*/

    odriveCAN.SetPosition(axes[1], kneeAngles[leftLegTarget] / 360.0 * 120);
          
    if(fabs(absDiff) <= posDiffThreshold) {
      leftKneeReady = true;
    }
  }
}

void moveJoint(Joint joint) {
  if(joint.readyToMove == false) {
    joint.posEstimate = odriveCAN.GetPosition(joint.node_id) / 120;  //motor position / gear ratio
    float absDiff = (joint.posEstimate - (joint.angles[joint.targetAngle] / 360.0)) * 120;
    #ifdef DEBUG
      Serial << "Pos est: " << joint.posEstimate * 120 << ", Target: " << joint.targetAngle << '
             "(" << joint.angles[joint.targetAngle] / 360 * 120 << ")" << '\n';
      Serial << "Velocity: " << odriveCAN.GetVelocity(joint.node_id) << "Pos diff: " << fabs(absDiff) 
             << '\n';
    #endif
    /*if(leftLegTarget == 0) {
      leftAnkleSpeed = (ankleAngles[49] - ankleAngles[leftLegTarget]) / 360 / segTime;
    } else {
      leftAnkleSpeed = (ankleAngles[leftLegTarget] - ankleAngles[leftLegTarget - 1]) / 360 / segTime;
    }*/

    odriveCAN.SetPosition(joint.node_id, joint.angles[joint.targetAngle] / 360.0 * 120);
          
    if(fabs(absDiff) <= posDiffThreshold) {
      joint.readyToMove = true;
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
      Serial << "Calibrating axis " << axes[inBytes[1]] << "..." << '\n';
      
      Serial << "Axis " << axes[inBytes[1]] << " motor calibration" << '\n';
      odriveCAN.RunState(axes[inBytes[1]], ODriveTeensyCAN::AXIS_STATE_MOTOR_CALIBRATION);
      motorError = odriveCAN.GetMotorError(axes[inBytes[1]]);
      if(motorError > 0) {
        Serial << "Motor calibration failed. Error code " << motorError << '\n';
        return;
      }

      Serial << "Axis " << axes[inBytes[1]] << " encoder calibration" << '\n';
      odriveCAN.RunState(axes[inBytes[1]], ODriveTeensyCAN::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
      encoderError = odriveCAN.GetEncoderError(axes[inBytes[1]]);
      if(encoderError > 0) {
        Serial << "Encoder calibration failed. Error code " << encoderError << '\n';
        return;
      }

      //TODO make calibrated var axis dependant
      calibrated = true;
      doneReading = false;
    } else if(inBytes[0] == 'm') {  //m1r45 -> move axis 1, 45 units
      if(!calibrated) {
        Serial.println("Axis not calibrated. Run calibration first");
        return;
      } else {
        odriveCAN.RunState(axes[inBytes[1]], ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
      }
      for(int i = 3;i < 32;i++) {
        if(inBytes[i] == '-') {
          isNeg = true;
        } else if(inBytes[i] != '\0') {
          inVal *= 10;
          inVal += inBytes[i] - '0';
        } else {
          if(isNeg) {
            inVal *= -1;
          }
          break;
        }
      }
      Serial << "Moving to: " << inVal << '\n';
      leftKneePosEstimate = odriveCAN.GetPosition(axes[inBytes[1]]);
      odriveCAN.SetPosition(axes[inBytes[1]], leftKneePosEstimate += inVal);
      inVal = 0;
      doneReading = false;
    } else if(inBytes[0] == 's') {  //stop everything
      for(int i = 0;i < sizeof(axes);i++) {
        odriveCAN.RunState(axes[i], ODriveTeensyCAN::AXIS_STATE_IDLE);
      }
      Serial.println("Movement stopped");
      programRunning = false;
    } else if(inBytes[0] == 'r') {  //run program
      if(!calibrated) {
        Serial.println("Axis not calibrated. Run calibration first");
        return;
      } else if(!allJointsHomed) {
        Serial.println("Joints not homed. Homing joints...");
        homeJoints();
      } else if(!atStart) {
        Serial.println("Moving joints to start position...");
        toStartPosition();
      } else {
        Serial.println("Starting program");
        programRunning = true;
      }
    } else if(inBytes[0] = 'h') {  //home joints
      Serial.println("Homing joints...");
      homeJoints();
    }
    doneReading = false;
  }
}

bool checkForErrors() {
  motorError = odriveCAN.GetMotorError(axes[1]);
  encoderError = odriveCAN.GetEncoderError(axes[1]);
  axisError = odriveCAN.GetAxisError(axes[1]);
  if(motorError != 0 || encoderError != 0 || axisError != 0) {
      programRunning = false;
      Serial << "Motor error: " << motorError << '\n';
      Serial << "Encoder error: " << encoderError << '\n';
      Serial << "Axis error: " << axisError << '\n';
      return true;
  }
  return false;
}

void homeJoints() {
  /*if(checkForErrors() == true) {
    return;
  }*/
  float leftHipDiff = 0;  //difference between current & home pos in revs
  float leftKneeDiff = 0;
  float leftAnkleDiff = 0;



  Serial.println("Homing joints...");
  
  while(!allJointsHomed) {
    if(Serial.available()) {
      readInput();
      return;
    }

    leftKneePotVal = analogRead(leftPosPinKnee);  //read current knee position
    leftKneeDiff = (leftKneePotVal - leftKneeOffset) * potValToDeg / 360;  //pos diff from deg to revs

    leftKneePosEstimate = odriveCAN.GetPosition(axes[1]);

    if(abs(leftKneeDiff) < posDiffThreshold) {  //if close enough
      leftKneeHomed = true;
    } else {
      odriveCAN.SetPosition(axes[1], leftKneePosEstimate - leftKneeDiff);
    }
    
    if(leftKneeHomed) {
      allJointsHomed = true;
    }
  }
  Serial.println("Homing sequence completed");
}

void toStartPosition() {
  /*if(checkForErrors() == true) {
    return;
  }*/
  if(!allJointsHomed) {
    Serial.println("Joints not homed. Home joints first");
    return;
  }
  bool leftHipAtStart = false;
  bool leftKneeAtStart = false;
  bool leftAnkleAtStart = false;


      
  while(!atStart) {
    if(Serial.available()) {
      readInput();
      return;
    }
   
    leftKneePosEstimate = odriveCAN.GetPosition(axes[1]);
    
    if(abs(leftKneePosEstimate - (kneeAngles[0] / 360)) <= posDiffThreshold) {
      leftKneeAtStart = true;
    } else {
      odriveCAN.SetPosition(axes[1], kneeAngles[0] / 360);
    }

    if(leftKneeAtStart) {
      atStart = true;
    }
  }
  Serial.println("At start position");
}
