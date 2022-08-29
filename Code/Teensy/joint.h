#ifndef Joint_h
#define Joint_h

#include <Arduino.h>
#include <ODriveTeensyCAN.h>

class Joint {
	private:
		
	public:
		Joint(int node_id, byte potPin, int potOffset, float *angles, int targetAngle, ODriveTeensyCAN *odriveCANPtr);
    int node_id;  //Odrive axis CAN node id
    byte potPin;  //analog input for joint output angle. Moved to odrive
    int potOffset;  //val of pot when joint at 0deg
    int targetAngle;  //angles array index for joint to move to next
    int startAngle;  //angles array index for joint to start at
    float targetPosition;  //joint angle to move to. Array element
    float targetVelocity = 6.0;  //slow start speed limit, just in case
    float velEstimate = 0.0  //store last known velocity, revs/s
    float posEstimate = 0.0;  //encoder reading in revs / gear ratio
    float *angles;  //pointer to angles array for this joint
		bool calibrated = true;  //has joint motor/encoder been calibrated. Done on odrive startup
    bool homed = true;  //home position is straight legs
    bool atStart = true;  //joint at start angle, ready for next?
    bool readyToMove = false;  //true means joint is at target position

    ODriveTeensyCAN *odriveCANPtr;  //reference to CAN bus object

    void ClearErrors();  //attempt to clear errors
    void GetErrors(uint64_t errorVals[3]);  //get motor(64bit), encoder(32bit), axis(32bit) errors

    void Calibrate();  //full calibration sequence
    void ClosedLoopCtrl();  //attempt to enter closed loop control
    void StopMotion();  //set axis state = idle
    
    float GetPotVal();  //pot val. Moved to odrive as GetAdcVoltage
		float GetPosition(); //get joint position in deg. Encoder estimate * 360 / gear ratio(120)
    float GetTargetPosition();  //get target encoder pos in revs
    float GetVelocity();  //get joint velocity in deg/s. motor velocity * 360 / gear ratio(120)
    uint32_t GetCurrentState();  //current axis state. ex IDLE or CLOSED_LOOP
    float GetIqMeasured();  //get current through motor

    void SetControlMode(int control_mode);  //change control mode
    void SetLinearCount(float linear_count);  //sets encoder.pos_estimate
    void SetPosition(float target_position);  //motor position in revs
    void SetPosition(float target_position, float ff_velocity);
    void SetVelocity(float target_velocity);  //in revs/s
    void SetTrajVelLimit(float velocity_limit);  //trajectory mode velocity limit in revs/s/s
    void SetLimits(float velocity_limit, float current_limit);  //set motor speed limit in revs/s, current in amps
};

#endif
