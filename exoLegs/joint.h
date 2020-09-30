#ifndef MY_JOINT_H
#define MY_JOINT_H

#include <Arduino.h>

class Joint {
	private:
		
	public:
		Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle);  //axis can node id on odrive
    int node_id;
    byte potPin;
    int potOffset;  //val of pot when joint at 0deg
    int targetAngle;
    float posEstimate = 0.0;  //encoder reading in revs
    float *angles;
		bool calibrated = false;
    bool homed = false;
    bool atStart = false;
    bool readyToMove = false;
    
    int GetPotPos();  //pot val
		void ManualMove(float rev); //for moving joint not in gait
};

#endif
