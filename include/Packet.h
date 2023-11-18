#ifndef _PACKET_H
#define _PACKET_H

#include <Arduino.h>

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
    MotorProperties_Power,
    MotorProperties_DirectDrive,
};

class Entity
{
    void SetProperty(int8_t property, int8_t value);
};

#endif  // _PACKET_H
