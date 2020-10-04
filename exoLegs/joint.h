#ifndef MY_JOINT_H
#define MY_JOINT_H

#include <Arduino.h>
#include <ODriveTeensyCAN.h>

class Joint {
	private:
		
	public:
		Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle, ODriveTeensyCAN *odriveCANPtr);  //axis can node id on odrive
    int node_id;
    byte potPin;
    int potOffset;  //val of pot when joint at 0deg
    int targetAngle;
    int startAngle;
    float targetPosition;
    float targetVelocity = 6.0;  //slow start speed limit, just in case
    float posEstimate = 0.0;  //encoder reading in revs
    float *angles;
		bool calibrated = true;
    bool homed = true;
    bool atStart = true;
    bool readyToMove = false;

    ODriveTeensyCAN *odriveCANPtr;  //reference to CAN bus object

    //int Heartbeat();  //heartbeat sent from odrive axis
    void ClearErrors();  //attempt to clear errors
    void GetErrors(uint32_t errorVals[3]);  //get motor, encoder, axis errors

    void Calibrate();  //full calibration sequence
    void ClosedLoopCtrl();  //attempt to enter closed loop control
    void StopMotion();  //set axis state = idle
    
    int GetPotPos();  //pot val
		float GetPosition(); //get joint position in revs
    float GetTargetPosition();  //get target joint pos using angles array
    float GetVelocity();  //get joint velocity in revs/s
    uint32_t GetCurrentState();  //current axis state. ex IDLE or CLOSED_LOOP

    void SetPosition(float target_position);
    void SetVelocityLimit(float velocity_limit);  //set speed in position control mode
};

#endif
