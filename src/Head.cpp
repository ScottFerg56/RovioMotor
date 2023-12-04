#include "Head.h"
#include "FLogger.h"

void Head::setProperty(Properties property, int16_t value)
{
    switch (property)
    {
    case Properties_Power:
        PowerChanged |= Power != value;
        Power = value;
        break;
    case Properties_Position:
        PositionChanged |= Position != value;
        Position = value;
        break;
    }
}

int16_t Head::getProperty(Properties property)
{
    switch (property)
    {
    case Properties_Power:
        return Power;
    
    case Properties_Position:
        return Position;
    }
    return -1;
}

bool Head::getPropertyChanged(Properties property)
{
    bool changed = false;
    switch (property)
    {
    case Properties_Power:
        changed = PowerChanged;
        PowerChanged = false;
        break;
    case Properties_Position:
        changed = PositionChanged;
        PositionChanged = false;
        break;
    }
    return changed;
}

bool Head::propertyToBot(Properties property)
{
    switch (property)
    {
    case Properties_Power:
        return true;
    default:
        return false;
    }
}

bool Head::propertyFromBot(Properties property)
{
    switch (property)
    {
    case Properties_Position:
    case Properties_Power:
        return true;
    default:
        return false;
    }
}

void Head::Init(int motorNum, byte pin)
{
    Motor = DriveMotor::MotorShield.getMotor(motorNum);
    if (Motor == nullptr)
        flogf("MotorShield getMotor failed");
}

void Head::Loop(unsigned long dmsec)
{
    //
    // analog values [0..4095]
    // the potentiometer on the Rovio head motor has a large 0 band several degrees right of vertical
    // and a large 4095 band several degrees left of center
    // there's a slight dimple on the left side of the slot
    // the pot spins freely/continuously
    // mapping the values from full retraction to full extension to [0..100]
    //                     encoder    mapped value
    // full retraction:       3000 ->   0
    // mid (level) extension: 1800 ->  50
    // full extension:         600 -> 100
    //
    // positive motor power values extend the head while negative values retract it
    //
    u_int16_t val = analogRead(37);
    int16_t pos = map(val, 730, 3100, 100, 0);
    pos = constrain(pos, 0, 100);
    //val = 100 - (val - 600 + 11) / 24;
    if (Position != pos)
    {
        Position = pos;
        PositionChanged = true;
    }
    if (Power > 0 && Position >= 100 || Power < 0 && Position <= 0)
    {
        Power = 0;
        PowerChanged = true;
    }

    //Serial.printf(">Head Encoder:%i\r\n", val);
    //Serial.printf(">Head Power:%i\r\n", Power);
    //Serial.printf(">Head Pos:%i\r\n", Position);
    if (PowerChanged)
    {
        Motor->setSpeed(abs(Power));
        Motor->run(Power == 0 ? RELEASE : (Power < 0 ? BACKWARD : FORWARD));
    }
}
