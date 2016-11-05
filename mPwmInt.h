/**add some modification by alezz
 ******************************************************************************
 *
 * @file       NazaInt.h
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

#include <inttypes.h>

#define PWM_INT_H_

#define MOTOR_LOWEST			1100			// trim throttle lowest here
#define MOTOR_HIGHEST		1900			// trim throttle highest here

#define PWM_SMOOTH              6 // SMOOTH data from PWM input

// uncomment to use sum-signal instead of single channels

#define PWM_PIN_MOTOR_1		4					// use pin  2 = PD4 = PCINT20	for M1
#define PWM_PIN_MOTOR_2		5					// use pin  9 = PD5 = PCINT21	for M2
#define PWM_PIN_MOTOR_3		6					// use pin 10 = PD6 = PCINT22	for M3
#define PWM_PIN_MOTOR_4		7					// use pin 11 = PD7 = PCINT23	for M4



#define PIN_READ_MOTOR_1		(PIND & 0b00010000)			// faster than digitalRead
#define PIN_READ_MOTOR_2		(PIND & 0b00100000)			// faster than digitalRead
#define PIN_READ_MOTOR_3		(PIND & 0b01000000)			// faster than digitalRead
#define PIN_READ_MOTOR_4		(PIND & 0b10000000)			// faster than digitalRead


#define CALL_CHECK_MOTOR_1		(port == 2 && mask & 0b00010000)	// call check macro
#define CALL_CHECK_MOTOR_2		(port == 2 && mask & 0b00100000)	// call check macro
#define CALL_CHECK_MOTOR_3		(port == 2 && mask & 0b01000000)	// call check macro
#define CALL_CHECK_MOTOR_4		(port == 2 && mask & 0b10000000)	// call check macro



void pwm_int_init(void);

int16_t motor1_us_get(void);
int16_t motor2_us_get(void);
int16_t motor3_us_get(void);
int16_t motor4_us_get(void);

