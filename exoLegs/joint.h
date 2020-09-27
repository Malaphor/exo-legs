#ifndef MY_JOINT_H
#define MY_JOINT_H

#include <Arduino.h>

class Joint (
	private:
		
	public:
		Joint(int node_id, byte potPin, float *angles, int targetAngle);  //axis can node id on odrive
    int node_id;
    byte potPin;
    int targetAngle;
    float posEstimate = 0.0;  //encoder reading in revs
		bool calibrated = false;
    bool readyToMove = false;
    
    int GetPotPos();  //pot val
		void ManualMove(float rev); //for moving joint not in gait
);

#endif
