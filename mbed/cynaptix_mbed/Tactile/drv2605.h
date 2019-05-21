#ifndef __DRV2605_H
#define __DRV2605_H

/**************************************************************
 *                                                            *
 * This is a library for the DRV2605 Haptic Driver            *
 * It has been adapted from the Adafruit_DRV2605_Library to   *
 * work with the Mbed microcontrollers instead of an Arduino. *
 *                                                            *
 * ----> https://github.com/adafruit/Adafruit_DRV2605_Library *
 *                                                            *
 **************************************************************/

#include "mbed.h"

#define DRV2605_ADDR 0x5A

#define DRV2605_REG_STATUS 0x00 // Status
#define DRV2605_REG_MODE 0x01 // Mode
#define DRV2605_MODE_INTTRIG  0x00 // Internal Trigger
#define DRV2605_MODE_EXTTRIGEDGE  0x01 // External Trigger (edge)
#define DRV2605_MODE_EXTTRIGLVL  0x02 // External Trigger (level)
#define DRV2605_MODE_ANALOGPWM  0x03 // Analog or PWM Input
#define DRV2605_MODE_AUDIOVIBE  0x04 // Audio-to-vibe
#define DRV2605_MODE_REALTIME  0x05 // Real Time Playback
#define DRV2605_MODE_DIAGNOS  0x06 // Diagnostics
#define DRV2605_MODE_AUTOCAL  0x07 // Callibration


#define DRV2605_REG_RTPIN 0x02 // Real-Time Playback Input

// Library Selection
#define DRV2605_REG_LIBRARY 0x03
#define DRV2605_LIB_EMPTY 0x00
#define DRV2605_LIB_A 0x01
#define DRV2605_LIB_B 0x02
#define DRV2605_LIB_C 0x03
#define DRV2605_LIB_D 0x04
#define DRV2605_LIB_E 0x05
#define DRV2605_LIB_LRA 0x06
#define DRV2605_LIB_F 0x07

// Waveform Sequencers
// bit 7 turns the sequence to a delay
// bits 6-0 are the id of the sequence to be played
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_WAVESEQ2 0x05
#define DRV2605_REG_WAVESEQ3 0x06
#define DRV2605_REG_WAVESEQ4 0x07
#define DRV2605_REG_WAVESEQ5 0x08
#define DRV2605_REG_WAVESEQ6 0x09
#define DRV2605_REG_WAVESEQ7 0x0A
#define DRV2605_REG_WAVESEQ8 0x0B

#define DRV2605_REG_GO 0x0C // Go register - see *go()* function
#define DRV2605_REG_OVERDRIVE 0x0D // Overdrive register (see datasheet)
#define DRV2605_REG_SUSTAINPOS 0x0E // Sustain positive time
#define DRV2605_REG_SUSTAINNEG 0x0F // Sustain negative time
#define DRV2605_REG_BREAK 0x10 // Brake time
#define DRV2605_REG_AUDIOCTRL 0x11 // Audio-to-vibe control
#define DRV2605_REG_AUDIOLVLIN 0x12 // Audio-to-vibe minimum input level
#define DRV2605_REG_AUDIOMAXIN 0x13 // Audio-to-vibe maximum input level
#define DRV2605_REG_AUDIOLVLOUT 0x14 // Audio-to-vibe minimum ouput level
#define DRV2605_REG_AUDIOMAXOUT 0x15 // Audio-to-vibe maximum output level
#define DRV2605_REG_RATEDV 0x16 // Rated voltage
#define DRV2605_REG_CLAMPV 0x17 // Voltage clamping
#define DRV2605_REG_AUTOCALCOMP 0x18 // Auto calibration compensation
#define DRV2605_REG_AUTOCALEMP 0x19 // Auto calibration back-emf result
#define DRV2605_REG_FEEDBACK 0x1A // Feedback control

// Control1 - contains STARTUP_BOOST, AC_COUPLE and DRIVE_TIME
#define DRV2605_REG_CONTROL1 0x1B 

// Control2 - contains BIDIR_INPUT, BRAKE_STABILISER, SAMPLE_TIME, 
// BLANKING_TIME and IDISS_TIME
#define DRV2605_REG_CONTROL2 0x1C

// Control3 - contains NG_THRESH, ERM_OPEN_LOOP, SUPPLY_COMP_DIS,
// DATA_FORMAT_RTP, LRA_DRIVE_MODE, N_PWM_ANALOG and LRA_OPEN_LOOP
#define DRV2605_REG_CONTROL3 0x1D

// Control4 - contains ZC_DET_TIME, AUTO_CAL_TIME, 
// OTP_STATUS and OTP_PROGRAM
#define DRV2605_REG_CONTROL4 0x1E

// Control5 - contains AUTO_OL_CNT, LRA_AUTO_OPEN_LOOP, PLAYBACK_INTERVAL,
// BLANKING_TIME and IDISS_TIME
#define DRV2605_REG_CONTROL5 0x1F

#define DRV2605_REG_LRA_PERIOD // LRA Open Loop Period
#define DRV2605_REG_VBAT 0x21 // V(BAT) Voltage Monitor
#define DRV2605_REG_LRARESON 0x22 // LRA Resonance Period (monitor)

class Drv2605 {
public:
    // Constructor
    // Pass in a pointer to an I2C class *i2c*
    Drv2605(I2C *i2c);

    // Init by default inisialises an lra driver
    // Set *lra* to false to drive an erm instead
    // *period_ms* is the period of oscillation of the driven LRA
    // For an ERM *period_ms* is the back emf sample rate
    void init(bool lra = true, float period_ms = 4.1666667);

    // Set the slot *slot* to contain the waveform *waveform*
    // The slots can be played in order with the *go()* function
    void set_waveform(char slot, char waveform);

    // Select the effect library *lib*
    void select_library(char lib);

    // Start playing the stored sequence of waveforms
    void go();
    // Abort waveform playback if it is in progress
    void stop();

    // Set the mode to be *mode*
    void set_mode(char mode);

    // Set the realtime value to be *value*
    void set_realtime_value(char value);
    
    // Write *val* to a register *reg*
    void write_register(char reg, char val);
    // Read the value stored in register *reg*
    char read_register(char reg);

private:
    // Pointer to I2C class
    I2C *p_i2c;

    char m_mode;

    // Setup an LRA driver
    void setup_lra(float period_ms);
    // Setup an ERM driver
    void setup_erm(float period_ms);
};

#endif // __DRV2605_H
