#include "Arduino.h"
#include "PVMInterface.h"

uint16_t icr = 0xffff;
void initPVM() {
  DDRB  |= _BV(PB1) | _BV(PB2);       /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                 /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS11);                  /* prescaler 1 */
  ICR1 = icr;                         /* TOP counter value (freeing OCR1A*/
}

void spinThrustFan(int speed) { //Speed is 0-255
  analogWrite(THRUST_FAN_PIN, speed);
}

void setServoAngle(float angle) {
  angle = angle > 180 ? 180 : angle;
  angle = angle < 0 ? 0 : angle;
  SERVO_PIN = 1150 + ((int) 3550 * (angle / 180.0f));
}

void spinLiftFan() {
  LIFT_FAN_PIN = 65535;
}

