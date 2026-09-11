/* board.h - pin map and fixed constants for the Synchronizer board

   Every pin here is taken from board/Synchronizer.kicad_sch (net names in
   brackets).  Do not change these without changing the PCB.
*/

/*
   Copyright (c) 2026 Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _BOARD_H
#define _BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- sense side ------------------------------------------------------- */

/* [/OSCIL] GPIO2 -> R3 -> Q1 (2N3904) -> R4 -> Q2 (2N3906) -> R5 -> tank.
   Driving this square wave at the tank's resonance excites the parallel
   LC formed by the sense coil (J3) and C3 || C4. */
#define GPIO_OSCIL          2

/* [/AMPLITUDE] GPIO26 / ADC0.  Envelope detector (R19, C11, D3, D4, R21, C9)
   fed from the LM358 second stage.  This is the slow control-path signal. */
#define GPIO_AMPLITUDE      26
#define ADC_CH_AMPLITUDE    0

/* [/OSC_SIGNAL] GPIO27 / ADC1.  The amplified tank waveform itself, before
   envelope detection.  Diagnostics only - it needs a fast capture. */
#define GPIO_OSC_SIGNAL     27
#define ADC_CH_OSC_SIGNAL   1

/* --- drive side ------------------------------------------------------- */

/* [/PULSE] GPIO4 -> R7 -> Q4 (2N3904) -> R9 -> gate of Q3 (IRF9540N, P-ch).
   Q4 on pulls the gate toward ground, VGS = -12 V, the FET conducts.
   So: GPIO4 HIGH energises the drive coil on J5.  It must idle LOW.

   The coil current runs through R6 (10R).  With an unknown coil resistance
   that resistor can be asked to dissipate several watts, so every pulse is
   bounded in both width and duty cycle - see drive.h. */
#define GPIO_PULSE          4

/* --- expansion -------------------------------------------------------- */

/* J2 carries GPIO5..GPIO15 for the GPS receiver that may be added later.
   GPIO12/GPIO13 are UART0 TX/RX (J2 pins 10 and 11). */
#define GPIO_GPS_TX         12
#define GPIO_GPS_RX         13
#define GPS_UART            uart0

/* --- the clock itself ------------------------------------------------- */

/* Measured from a three-minute recording on 2026-09-10: the escapement
   pattern repeats every 0.857030 s, which is 8400 beats per hour within the
   measurement error.  Two beats per full swing, so the sense coil - which
   sits at one extreme of the swing - sees the bob once per full period.

   8400 bph = 4200 full swings/hour = 100800 swings/day. */
#define DEFAULT_BEATS_PER_HOUR   8400u
#define BEATS_PER_SWING          2u

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H */
