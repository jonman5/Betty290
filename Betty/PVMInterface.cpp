#include "Arduino.h"
#include <arduino-timer.h>
#include "PVMInterface.h"
#include "sensor.h"


uint16_t icr = 0xffff;


void initPVM() {
  DDRB  |= _BV(PB1) | _BV(PB2);       /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                 /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS11);                  /* prescaler 1 */
  ICR1 = icr;                         /* TOP counter value (freeing OCR1A*/
  auto timer = timer_create_default(); //create a timer instance *timer*
  hovercraft.leftSensor = createSensor(PD3);
  hovercraft.rightSensor = createSensor(PD4);
}
void spinThrustFan() {
  analogWrite(THRUST_FAN_PIN, 200);
}

void setServoAngle(int angle) {
  angle = angle > 180 ? 180 : angle;
  angle = angle < 0 ? 0 : angle;
  SERVO_PIN = 1100 + ((int) 3550 * (((float) angle) / 180.0f));
}

void setServoPin(int value) {
  SERVO_PIN = value;
}

void spinLiftFan() {
  LIFT_FAN_PIN = 65535;
}

void moveForward(){
  if (turning) return;
  setServoAngle(90);
  delay(100);
  spinLiftFan();
  delay(100);
  spinThrustFan();
}

void stabilize(){
  if (turning) return;
  setServoAngle(60);
}

void turn(){
  if (turning){
    setServoAngle(135);
  }
}

void stopTurn(){
  turning=false;
  moveForward();
}

void checkSensors(){
  updateSensorDistance(&hovercraft.leftSensor);
  updateSensorDistance(&hovercraft.rightSensor);
  if (hovercraft.rightSensor.distanceCM < thresholdDist){
    turning = true;
    turn();
  }
}
