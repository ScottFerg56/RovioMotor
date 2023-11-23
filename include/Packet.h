#ifndef _PACKET_H
#define _PACKET_H

#include <Arduino.h>

enum Entities : uint8_t
{
    Entities_None,
    Entities_LeftMotor,
    Entities_RightMotor,
    Entities_RearMotor,
};

const Entities firstEntity = Entities_LeftMotor;
const Entities  lastEntity = Entities_RearMotor;

// prefix ++
inline Entities& operator++(Entities& k)
{
    k = (Entities)((int)k + 1);
    return k;
}

// postfix ++
inline Entities operator++(Entities& k, int)
{
    Entities o = k;
    k = (Entities)((int)k + 1);
    return o;
}

enum Properties : uint8_t
{
    Properties_None,
    Properties_Goal,
    Properties_RPM,
    Properties_Power,
    Properties_DirectDrive,
    // controller-only properties:
    Properties_ControlMode,
};

const Properties firstProperty = Properties_Goal;
const Properties lastProperty = Properties_DirectDrive;

// prefix ++
inline Properties& operator++(Properties& k)
{
    k = (Properties)((int)k + 1);
    return k;
}

// postfix ++
inline Properties operator++(Properties& k, int)
{
    Properties o = k;
    k = (Properties)((int)k + 1);
    return o;
}

class Entity
{
public:
    virtual void setProperty(Properties property, int16_t value) = 0;
    virtual int16_t getProperty(Properties property) = 0;
    virtual bool getPropertyChanged(Properties property) = 0;
};

struct Packet
{
    Entities entity;
    Properties property;
    int16_t value;
};

#endif  // _PACKET_H
