#define KNOBS 1
#define PLOT 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <QuadEncoder.h>
#include "DriveMotor.h"
#ifdef KNOBS
#include "ScaledKnob.h"
#endif

bool DoKnobs = true;
bool DoPlot = true;
unsigned long LastTime = 0;

// Right Motor has encoder on opposite side from other motors, so encoder pins must be reversed WRT motor polarity
Encoders LeftMotorEncoder(6,5);
Encoders RightMotorEncoder(A4,A5);
Encoders RearMotorEncoder(A2,A3);

Adafruit_seesaw SeeSaw;
seesaw_NeoPixel SSPixel = seesaw_NeoPixel(4, 18, NEO_GRB + NEO_KHZ800);

void dump(const char* name, float v)
{
  Serial.print(name);
  Serial.print(":");
  Serial.println(v, 3);
}

#ifdef PLOT
void plot(const char* name, float v)
{
  if (!DoPlot)
    return;
  Serial.print(">");
  Serial.print(name);
  //Serial.print(":");
  //Serial.print(LastTime);
  Serial.print(":");
  Serial.println(v, 3);
}
#endif

float Map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

#ifdef KNOBS
ScaledKnob KnobKp(0, 12, 0, 50, 0.1);
ScaledKnob KnobLeft( 1, 14, -100, 100, 5);
ScaledKnob KnobRight(2, 17, -100, 100, 5);
ScaledKnob KnobRear( 3,  9, -100, 100, 5);
#endif

Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
DriveMotor LeftMotor;
DriveMotor RightMotor;
DriveMotor RearMotor;

void setup()
{
  LastTime = millis();
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("In setup");
  #ifdef KNOBS
  Serial.println("KNOBS");
  #endif
  #ifdef PLOT
  Serial.println("PLOT");
  #endif

  if (!SeeSaw.begin(0x49) || !SSPixel.begin(0x49))
  {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }

  SSPixel.setBrightness(50);

  MotorShield.begin();
  LeftMotor.Init( MotorShield.getMotor(4), &LeftMotorEncoder);
  RightMotor.Init(MotorShield.getMotor(3), &RightMotorEncoder);
  RearMotor.Init( MotorShield.getMotor(1), &RearMotorEncoder);

#ifdef KNOBS
  KnobKp.Init(&SeeSaw, &SSPixel, RearMotor.Kp);
  KnobLeft.Init(&SeeSaw, &SSPixel, LeftMotor.SpeedGoal);
  KnobRight.Init(&SeeSaw, &SSPixel, RightMotor.SpeedGoal);
  KnobRear.Init(&SeeSaw, &SSPixel, RearMotor.SpeedGoal);

  KnobKp.SetColor(0, 0, 128);
  KnobLeft.SetColor(0, 0, 128);
  KnobRight.SetColor(0, 0, 128);
  KnobRear.SetColor(0, 0, 128);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Setup complete");
}

void loop()
{
  unsigned long msec = millis();
  unsigned long dmsec = msec - LastTime;
  if (dmsec >= 100)
  { 
    LastTime = msec;

#ifdef KNOBS
    if (DoKnobs)
    {
      KnobKp.Sample();
      KnobLeft.Sample();
      KnobRight.Sample();
      KnobRear.Sample();

      if (KnobKp.Pressed() == ScaledKnob::Presses::Press)
          KnobKp.SetValue(0);
      if (KnobLeft.Pressed() == ScaledKnob::Presses::Press)
          KnobLeft.SetValue(0);
      if (KnobRight.Pressed() == ScaledKnob::Presses::Press)
          KnobRight.SetValue(0);
      if (KnobRear.Pressed() == ScaledKnob::Presses::Press)
          KnobRear.SetValue(0);

      LeftMotor.SpeedGoal = KnobLeft.GetValue();
      LeftMotor.Kp = KnobKp.GetValue();
      RightMotor.SpeedGoal = KnobRight.GetValue();
      RightMotor.Kp = KnobKp.GetValue();
      RearMotor.SpeedGoal = KnobRear.GetValue();
      RearMotor.Kp = KnobKp.GetValue();
    }
#endif

    LeftMotor.Loop(dmsec);
    RightMotor.Loop(dmsec);
    RearMotor.Loop(dmsec);

#ifdef PLOT
    plot("LeftGoal", LeftMotor.SpeedGoal);
    plot("RightGoal", RightMotor.SpeedGoal);
    plot("RearGoal", RearMotor.SpeedGoal);
    plot("LeftCount", LeftMotor.Count);
    plot("RightCount", RightMotor.Count);
    plot("RearCount", RearMotor.Count);
    plot("LeftRPM", LeftMotor.RPM);
    plot("RightRPM", RightMotor.RPM);
    plot("RearRPM", RearMotor.RPM);
    //plot("Kp", RearMotor.Kp);
    //plot("Ki", RearMotor.Ki);
    //plot("Kd", RearMotor.Kd);
    //plot("Error", RearMotor.Error);
    //plot("ErrorSum", RearMotor.ErrorSum);
    //plot("LeftOutput", LeftMotor.Output);
    //plot("RightOutput", RightMotor.Output);
    //plot("RearOutput", RearMotor.Output);
    plot("LeftPower", LeftMotor.Power);
    plot("RightPower", RightMotor.Power);
    plot("RearPower", RearMotor.Power);
#endif
  }
}
