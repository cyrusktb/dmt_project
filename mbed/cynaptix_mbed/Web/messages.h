#ifndef __MESSAGES_H__
#define __MESSAGES_H__

/**********************************************
 * This contains a bunch of definitions used  *
 * by any networking class to know standard   *
 * message contents                           *
 **********************************************/

// This is used in a few other files, so make it 
// super clear where things come from
namespace Msgs {

// Message start byte: Followed by: <message> MSG_END
const char MSG_STRT = 0xFF;

// Message end byte: Followed by: <N/A>
const char MSG_END = 0xFE;

// Target motor position (mm / deg): Followed by: <motor_num> <32 bit float>
const char TRGT_MTR_POS = 0x03;

// Measured motor torque (Nm): Followed by: <motor_num> <32 bit float>
const char MTR_TRQ = 0x04;

// Measured motor position (mm / deg): Followed by: <motor_num> <32 bit float>
const char MTR_POS = 0x05;

// Vibration motor strengths (%): Followed by: <9 bytes>
const char VIB_STR = 0x06;

// Motor names
const char THUMB = 0x06;
const char INDEX = 0x07;
const char MIDDLE = 0x08;

};

#endif // __MESSAGES_H__
