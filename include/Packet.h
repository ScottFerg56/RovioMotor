#ifndef _PACKET_H
#define _PACKET_H

#include <Arduino.h>

enum PacketCmds
{
    CmdNone,
    // int8_t -100 to 100
    CmdLeftMotorGoal,
    CmdRightMotorGoal,
    CmdRearMotorGoal,
    CmdLeftMotorRPM,
    CmdRightMotorRPM,
    CmdRearMotorRPM,
};

enum Entities
{
    Entities_LeftMotor,
    Entities_RightMotor,
    Entities_RearMotor,
    Entities_AllMotors,
};

enum MotorProperties
{
    MotorProperties_Goal,
    MotorProperties_RPM,
    MotorProperties_DirectDrive,
};

class Entity
{
    void SetProperty(int8_t property, int8_t value);
};

#endif  // _PACKET_H
