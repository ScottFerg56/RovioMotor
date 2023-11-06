#ifndef _PACKET_H
#define _PACKET_H

//#include <Arduino.h>

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

#endif  // _PACKET_H
