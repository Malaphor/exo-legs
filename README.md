# The Exoskeleton Project

## Overview

This project is a lower limb exoskeleton with 6 active joints. Its built with 3D printed and off-the-shelf parts. The aim is to get a basic, robust platform up and running for a comparatively low cost so that anyone that wants to can build it, use it and modify it however they wish. The exoskeleton is controlled with a [Teensy 4](https://www.pjrc.com/teensy/) and [ODrive 3.6](https://odriverobotics.com/shop/odrive-v36).

Requires [ODriveTeensyCAN](https://github.com/Malaphor/ODriveTeensyCAN) and [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4).

## Building

In the Documentation folder you will find assembly instructions and reference material for each of the parts required to build this project. There are separate folders for CAD files and STL files. The STL files are ready to print while the CAD files are there if you wish to modify any of the parts. Some off-the-shelf parts will need to be modified or adjusted to fit. For example, you may need to sand the inside of a bearing to fit around a motor or cut the aluminum extrusion to a specific length.

## Code

There are 3 subfolders in the Code folder. The code that runs on the Teensy is the brains of the exoskeleton. It handles gait, communication with motor controllers and sensors and also transfers data to the GUI. There is modified ODrive firmware that you'll need to flash to your ODrives to get voltage feedback from each joint potentiometer and to read some of the error flags in the heartbeat CAN message. Finally, the 3rd folder contains code for the GUI. This part is written in [Processing](https://processing.org/) and meant to be portable to run on whichever system you're using to control the exoskeleton, whether it be a tablet or desktop or something else.

# Mechanical Design

There are 6 active joints, 1 for each hip, knee and ankle. All of the joint actuators are designed to be extremely similar so once you get one joint working you can just print 5 more. The motor used here is a GARTT ML5210 brushless DC motor coupled to a size 17 120:1 strain wave gear (this is the most expensive part). This pair allows for up to 35Nm nominal torque and much more instantaneous torque. Both motor and gear are supported by large deep groove ball bearings and sandwiched between 3D printed parts. Between each joint and around the lower back are pieces of 20x40 aluminum extrusion that have been cut to length.

![Anatomy of a Joint](/Documentation/anatomyOfJoint.jpg)

# Electronics

A Teensy 4 MCU controls 3 v3.6 24V ODrives with a [SN65HVD230](https://www.ti.com/product/SN65HVD230) CAN transciever break out board operating at 1Mbps for communicating between them. Each ODrive controls 2 motors with an AS5047P eval board in ABI mode for motor commutation. An NTC 3950 thermistor was attached to the motor stator with thermal epoxy and wired into the ODrive to prevent overheating. The output of each joint also has a 3-turn potentiometer (3547S-1AA-103A) that measures the joint position with voltage feedback from 0V (min angle) to 3.3V (max angle). Motor position given by the AS5047P multiplied by the gear ratio gives joint position for the gait while the potentiometer is used for getting the startup position and homing each of the joints. Power to the system is currently supplied by a generic PSU like that you might find on a 3D printer.

![Electrical Diagram](/Documentation/electricalDiagram.jpg)

*Note: If you're connecting an ODrive or Teensy to a PC a USB isolator is recommended

# Software

The initial control strategy is meant to be pretty straightforward. For each joint there's an array of 50 data points (from The Biomechanics and Motor Control of Human Gait, Winter 1987) that get looped through and sent to the ODrives as position commands. There's a separate header and implementation file that defines a Joint object which is used to keep track of various data related to each hip, knee and ankle joint. A section of the code is dedicated to reading CAN messages from the ODrives with the help of the ODriveTeensyCAN library. Data transfer between the Teensy and a PC is split into 2 types: GUI and not GUI (controlled by a variable definition). When in GUI mode data and commands are sent back and forth between the Teensy and the GUI. Otherwise you may use the Arduino console to input commands and read debug info. The code is well commented to make it easier to follow along.