#include "joint.h"

float gearRatio = 120.0;

Joint::Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle, ODriveTeensyCAN *odriveCANPtr) {
  this->node_id = node_id;
  this->potPin = potPin;
  this->potOffset = potOffset;
  this->angles = angles;
  this->targetAngle = targetAngle;
  this->odriveCANPtr = odriveCANPtr;
}
/*
int Joint::Heartbeat() {
  return odriveCANPtr->Heartbeat();
}*/

void Joint::ClearErrors() {
  odriveCANPtr->ClearErrors(this->node_id);
}

void Joint::GetErrors(uint32_t errorVals[3]) {
  memset(errorVals, 0, sizeof(errorVals));  //sets all values to 0
  
  errorVals[0] = odriveCANPtr->GetMotorError(this->node_id);
  errorVals[1] = odriveCANPtr->GetEncoderError(this->node_id);
  errorVals[2] = odriveCANPtr->GetAxisError(this->node_id);
}

void Joint::Calibrate() {
  odriveCANPtr->RunState(this->node_id, ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
}

void Joint::ClosedLoopCtrl() {
  odriveCANPtr->RunState(this->node_id, ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void  Joint::StopMotion() {
  odriveCANPtr->RunState(this->node_id, ODriveTeensyCAN::AXIS_STATE_IDLE);
}

int Joint::GetPotPos() {
  return analogRead(potPin);
}

float Joint::GetPosition() {
  this->posEstimate = odriveCANPtr->GetPosition(this->node_id) / gearRatio;
  return this->posEstimate;
}

float Joint::GetTargetPosition() {
  this->targetPosition = this->angles[this->targetAngle] / 360.0 * gearRatio;
  return targetPosition;
}

float Joint::GetVelocity() {
  return odriveCANPtr->GetVelocity(this->node_id) / gearRatio;
}

uint32_t Joint::GetCurrentState() {
  return odriveCANPtr->GetCurrentState(this->node_id);
}

void Joint::SetPosition(float target_position) {
  odriveCANPtr->SetPosition(this->node_id, target_position);
}

void Joint::SetVelocityLimit(float velocity_limit) {
  odriveCANPtr->SetVelocityLimit(this->node_id, velocity_limit);
}
