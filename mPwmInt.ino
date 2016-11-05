/**add some modification by alezz
 ******************************************************************************
 *
 * @file       mPwmInt.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements RC-RX channel detection and NAZA artificial horizon 
 *             using interrupts.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


// !!! For using this, you have to solder a little bit on the MinimOSD !!!


#include "mPwmInt.h"


#define PWM_INT_MINIMAL


// see http://playground.arduino.cc/Main/PcInt

#include "pins_arduino.h"

/*
 * an extension to the interrupt support for arduino.
 * add pin change interrupts to the external interrupts, giving a way
 * for users to have interrupts drive off of any pin.
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 */

/*
 * Theory: all IO pins on Atmega168 are covered by Pin Change Interrupts.
 * The PCINT corresponding to the pin must be enabled and masked, and
 * an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
 * must use some logic to actually implement a per-pin interrupt service.
 */

/* Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 */

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

#ifndef PWM_INT_MINIMAL
static int PCintMode[24];
typedef void (*voidFuncPtr)(void);
volatile static voidFuncPtr PCintFunc[24] = { NULL };
#endif

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

#ifndef PWM_INT_MINIMAL
  uint8_t slot;
// -- Fix by Baziki. In the original sources it was a little bug, which cause analog ports to work incorrectly.
  if (port == 1) {
     slot = port * 8 + (pin - 14);
  }
  else {
     slot = port * 8 + (pin % 8);
  }
// --Fix end
  PCintMode[slot] = mode;
  PCintFunc[slot] = userFunc;
#endif

  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= (0x01 << port);
}


// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
static void PCint(uint8_t port) {
  uint8_t curr;
  uint8_t mask;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask the pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }

  if (CALL_CHECK_MOTOR_1)	int_motor1();
  if (CALL_CHECK_MOTOR_2)	int_motor2();
  if (CALL_CHECK_MOTOR_3)	int_motor3();
  if (CALL_CHECK_MOTOR_4)	int_motor4();

}

/*
SIGNAL(PCINT0_vect) {
  PCint(0);
}
SIGNAL(PCINT1_vect) {
  PCint(1);
}
*/
/*
SIGNAL(PCINT2_vect) {
  PCint(2);
}*/


volatile long motor1_start;				// start time
volatile long motor1_pulse		= 1500;		// pulse duration

volatile long motor2_start;			// start time
volatile long motor2_pulse	= 1500;		// pulse duration

volatile long motor3_start;				// start time
volatile long motor3_pulse		= 1500;		// pulse duration
//volatile long motor3_stop;
//volatile long motor3_pause		= 1500;		// pulse duration

volatile long motor4_start;				// start time
volatile long motor4_pulse		= 1500;		// pulse duration

#if (PWM_SMOOTH > 1)
static uint8_t ind_1 = 0;
static long raw_1[PWM_SMOOTH], rsum_1;
static uint8_t ind_2 = 0;
static long raw_2[PWM_SMOOTH], rsum_2;
static uint8_t ind_3 = 0;
static long raw_3[PWM_SMOOTH], rsum_3;
static uint8_t ind_4 = 0;
static long raw_4[PWM_SMOOTH], rsum_4;
#endif

void int_motor1(void) {
	if (PIN_READ_MOTOR_1)
		motor1_start = micros();				// positive edge: start
	else
		motor1_pulse = micros() - motor1_start;		// negative edge: calculate pulsewidth
}

void int_motor2(void) {
	if (PIN_READ_MOTOR_2)
		motor2_start = micros();				// positive edge: start
	else
		motor2_pulse = micros() - motor2_start;		// negative edge: calculate pulsewidth
}
/*
void int_motor3(void) {
	if (PIN_READ_MOTOR_3) {
		motor3_start = micros();				// positive edge: start
                motor3_pause = micros() - motor3_stop;
        }
	else {
		motor3_stop = micros();
                motor3_pulse = micros() - motor3_start;		// negative edge: calculate pulsewidth
              }
}
*/

void int_motor3(void) {
	if (PIN_READ_MOTOR_3)
		motor3_start = micros();				// positive edge: start
	else
		motor3_pulse = micros() - motor3_start;		// negative edge: calculate pulsewidth
}

void int_motor4(void) {
	if (PIN_READ_MOTOR_4)
		motor4_start = micros();				// positive edge: start
	else
		motor4_pulse = micros() - motor4_start;		// negative edge: calculate pulsewidth
}



void pwm_int_init(void)
{
	pinMode(PWM_PIN_MOTOR_1, INPUT);
	digitalWrite(PWM_PIN_MOTOR_1, HIGH);				// turn on pullup resistor
	PCattachInterrupt(PWM_PIN_MOTOR_1, int_motor1, CHANGE);

	pinMode(PWM_PIN_MOTOR_2, INPUT);
	digitalWrite(PWM_PIN_MOTOR_2, HIGH);				// turn on pullup resistor
	PCattachInterrupt(PWM_PIN_MOTOR_2, int_motor2, CHANGE);
	
	pinMode(PWM_PIN_MOTOR_3, INPUT);
	digitalWrite(PWM_PIN_MOTOR_3, HIGH);				// turn on pullup resistor
	PCattachInterrupt(PWM_PIN_MOTOR_3, int_motor3, CHANGE);

	pinMode(PWM_PIN_MOTOR_4, INPUT);
	digitalWrite(PWM_PIN_MOTOR_4, HIGH);				// turn on pullup resistor
	PCattachInterrupt(PWM_PIN_MOTOR_4, int_motor4, CHANGE);

}


// throttle in percent
int8_t motor1_percent_get(void)
{
    #if PWM_SMOOTH == 1
      return constrain((int8_t)((float)(motor1_pulse - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #else
      long r= motor1_pulse; 
      rsum_1 += r;
      rsum_1 -= raw_1[ind_1];
      raw_1[ind_1++] = r;
      ind_1 %= PWM_SMOOTH;
      r = rsum_1 / PWM_SMOOTH;
      return constrain((int8_t)((float)(r - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #endif	
  
}

int8_t motor2_percent_get(void)
{
    #if PWM_SMOOTH == 1
      return constrain((int8_t)((float)(motor2_pulse - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #else
      long r= motor2_pulse; 
      rsum_2 += r;
      rsum_2 -= raw_2[ind_2];
      raw_2[ind_2++] = r;
      ind_2 %= PWM_SMOOTH;
      r = rsum_2 / PWM_SMOOTH;
      return constrain((int8_t)((float)(r - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #endif
}

int8_t motor3_percent_get(void)
{
    #if PWM_SMOOTH == 1
      return constrain((int8_t)((float)(motor3_pulse - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #else
      long r= motor3_pulse; 
      rsum_3 += r;
      rsum_3 -= raw_3[ind_3];
      raw_3[ind_3++] = r;
      ind_3 %= PWM_SMOOTH;
      r = rsum_3 / PWM_SMOOTH;
      return constrain((int8_t)((float)(r - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #endif
}

int8_t motor4_percent_get(void)
{
    #if PWM_SMOOTH == 1
      return constrain((int8_t)((float)(motor4_pulse - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #else
      long r= motor4_pulse; 
      rsum_4 += r;
      rsum_4 -= raw_4[ind_4];
      raw_4[ind_4++] = r;
      ind_4 %= PWM_SMOOTH;
      r = rsum_4 / PWM_SMOOTH;
      return constrain((int8_t)((float)(r - MOTOR_LOWEST) / (float)(MOTOR_HIGHEST - MOTOR_LOWEST) * 100.0 + 0.5), 0, 100);
    #endif
}


// motor in micro seconds
int16_t motor1_us_get(void)
{
	return (int16_t) motor1_pulse;
}

// motor in micro seconds
int16_t motor2_us_get(void)
{
	return (int16_t) motor2_pulse;
}

// motor in micro seconds
int16_t motor3_us_get(void)
{
	return (int16_t) motor3_pulse;
        //return (int16_t) motor3_pause;
        //return (int16_t) motor3_pulse + motor3_pause;
}

// motor in micro seconds
int16_t motor4_us_get(void)
{
	return (int16_t) motor4_pulse;
}

