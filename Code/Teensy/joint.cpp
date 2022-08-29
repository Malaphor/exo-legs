#include "Joint.h"

float motorGearRatio = 120.0;
float potGearRatio = 4.09;  //1.715V @ 0deg, 2.475V @ 90deg, 3.3/4 = 0.825, 0.825/90 = 0.0091V per deg

Joint::Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle, ODriveTeensyCAN *odriveCANPtr) {
  this->node_id = node_id;  //odrive axis CAN node id
  this->potPin = potPin;  //gpio pin on odrive
  this->potOffset = potOffset;  //val of pot in volts when joint at 0deg
  this->angles = angles;  //pointer to array of joint angles
  this->targetAngle = targetAngle;  //angle joint is moving to next
  this->startAngle = targetAngle;  //angle joint  at beginning of gait
  this->odriveCANPtr = odriveCANPtr;  //pointer to CANbus object
}

void Joint::ClearErrors() {
  odriveCANPtr->ClearErrors(this->node_id);
}

void Joint::GetErrors(uint64_t errorVals[3]) {
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

float Joint::GetPotVal() {
  return odriveCANPtr->GetADCVoltage(this->node_id, this->potPin);
}

float Joint::GetPosition() {
  this->posEstimate = odriveCANPtr->GetPosition(this->node_id);  //encoder position in revs
  return this->posEstimate;
}

float Joint::GetTargetPosition() {
  this->targetPosition = this->angles[this->targetAngle] * motorGearRatio / 360.0;  //target encoder position in revs
  return targetPosition;
}

float Joint::GetVelocity() {
  return odriveCANPtr->GetVelocity(this->node_id) * 360 / motorGearRatio;  //output joint velocity
}

uint32_t Joint::GetCurrentState() {
  return odriveCANPtr->GetCurrentState(this->node_id);
}

float Joint::GetIqMeasured() {
  return odriveCANPtr->GetIqMeasured(this->node_id);
}

void Joint::SetControlMode(int control_mode) {
  odriveCANPtr->SetControllerModes(this->node_id, control_mode, 1);  //1 == input mode passthrough
}

void Joint::SetLinearCount(float linear_count) {
  odriveCANPtr->SetLinearCount(this->node_id, linear_count);
}

void Joint::SetPosition(float target_position) {
  odriveCANPtr->SetPosition(this->node_id, target_position);  //move motor to encoder position in revs
}

void Joint::SetPosition(float target_position, float ff_velocity) {
  odriveCANPtr->SetPosition(this->node_id, target_position, ff_velocity);
}

void Joint::SetVelocity(float target_velocity) {
  odriveCANPtr->SetVelocity(this->node_id, target_velocity);  //input velocity in velocity control mode, in revs/s
}

void Joint::SetTrajVelLimit(float velocity_limit) {
  odriveCANPtr->SetTrajVelLimit(this->node_id, velocity_limit);
}

void Joint::SetLimits(float velocity, float current) {
  odriveCANPtr->SetLimits(this->node_id, velocity, current);  //set motor velocity limit, current limit
}
