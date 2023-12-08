#include "Head.h"
#include "FLogger.h"

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
    const int16_t posScale = 50;
    // a range of [0..100] is nice,
    // but the fluctuations in analog readings causes too much jitter
    // so use a smaller range and rescale to reduce resolution
    int16_t pos = map(val, 730, 3100, posScale, 0);
    pos = constrain(pos, 0, posScale) * (100 / posScale);
    Position.Set(pos);
    // stop the head motor at the top and bottom limits
    if (Power.Get() > 0 && pos >= 100 || Power.Get() < 0 && pos <= 0)
        Power.Set(0);

    //Serial.printf(">Head Encoder:%i\r\n", val);
    //Serial.printf(">Head Power:%i\r\n", Power);
    //Serial.printf(">Head Pos:%i\r\n", Position);
    if (Power.IsChanged())
    {
        Motor->setSpeed(abs(Power.Get()));
        Motor->run(Power.Get() == 0 ? RELEASE : (Power.Get() < 0 ? BACKWARD : FORWARD));
    }
}
