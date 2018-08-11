#ifndef __DEFINES_H__
#define __DEFINES_H__

/* includes */
#include "stdbool.h"

/*----------------------------------------------*/
/*             EXTI lock variable               */
/*----------------------------------------------*/
/* to avoid reentering to key matrix loop again */
/* this variable acts as a gate from accidental */
/*             interrupt triggers               */
/*----------------------------------------------*/
extern bool gateKeeper;

/*----------------------------------------------*/
/* used for debugging purposes, remove in final */
/* release.                                     */
/*----------------------------------------------*/
extern uint8_t dummy;

/*---------------------*/
/* Function Prototypes */
/*---------------------*/
void Transmit(uint8_t);
void TransmitMidi(uint8_t, uint8_t);

/*-----------------------*/
/* EXTI Lock definitions */
/*-----------------------*/
#define __locked   1
#define __unlocked 0

/*-----------------------*/
/* MIDI note definitions */
/*-----------------------*/
#define A4 0x45
#define Bb4 0x46
#define B4 0x47
#define C4 0x48
#define Db4 0x49
#define D4 0x50
#define Eb4 0x51
#define E4 0x52
#define F4 0x53

/*----------------------*/
/* velocity Definitions */
/*----------------------*/
#define VELOCITY_MAX 0x7f


#endif
