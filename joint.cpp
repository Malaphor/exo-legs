#include "joint.h"

Joint::Joint(int node_id, byte potPin, float *angles, int targetAngle) {
  this->node_id = node_id;
  this->potPin = potPin;
  this->angles = angles;
  this->targetAngle = targetAngle;
}

int GetPotPos() {
  return analogRead(potPin);
}

void ManualMove() {
  
}
