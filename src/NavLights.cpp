#include "NavLights.h"
#include "LEDAnimator.h"
#include "FLogger.h"
#include "AnimFwd.h"
#include "AnimCylon.h"
#include "AnimGreen.h"

#define DATA_PIN 12
#define NUM_LEDS 6

LEDAnimator<WS2811, DATA_PIN, GRB> Animator(NUM_LEDS);

void NavLights::Init()
{
    FastLED.setCorrection(TypicalLEDStrip);
    Brightness.Set(255);
    Iterations.Set(4);
    Animation.Set(Animation_Green);
    Animator.Init();
}

void NavLights::Loop()
{
    if (Brightness.IsChanged())
        Animator.ChangeBrightness(Brightness.Get());
    if (Iterations.IsChanged())
        Animator.ChangeIterations(Iterations.Get());
    if (Animation.IsChanged())
    {
        uint16_t iters = Iterations.Get();
        uint8_t bright = Brightness.Get();
        //flogd("anim: %i  bright: %i  iters: %i", Animation.Get(), bright, iters);
        switch ((Animations)Animation.Get())
        {
        case Animation_None:
            Animator.SetAnim(nullptr, 0, 0, 0);
            break;
        case Animation_Green:
            Animator.SetAnim(&AnimGreenAnim, 1000, bright, iters, 10, 10);
            break;
        case Animation_Cylon:
            Animator.SetAnim(&AnimCylonAnim, 75, bright, iters);
            break;
        case Animation_Fwd:
            Animator.SetAnim(&AnimFwdAnim, 500, bright, iters, 20, 20);
            break;
        }
    }
    if (!Animator.Loop())
        Animation.Set(Animation_None);
}
