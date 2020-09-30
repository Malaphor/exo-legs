#include "joint.h"

Joint::Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle) {
  this->node_id = node_id;
  this->potPin = potPin;
  this->potOffset = potOffset;
  this->angles = angles;
  this->targetAngle = targetAngle;
}

int Joint::GetPotPos() {
  return analogRead(potPin);
}

void ManualMove() {
  
}
