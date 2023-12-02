#ifndef _HEAD_H
#define _HEAD_H

#include <DriveMotor.h>

class Head : Entity
{
protected:
    Adafruit_DCMotor* Motor;
    uint8_t potPin;
public:
    int16_t Power = 0;
    bool PowerChanged = false;
    int16_t Position = 0;
    bool PositionChanged = false;

    void Init(int motorNum, byte pin);
    void Loop(unsigned long dmsec);

    void setProperty(Properties property, int16_t value);
    int16_t getProperty(Properties property);
    bool getPropertyChanged(Properties property);
    bool propertyToBot(Properties property);
    bool propertyFromBot(Properties property);
};

#endif // _HEAD_H
