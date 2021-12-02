#include "Arduino.h"
#include "PVMInterface.h"

struct Hovercraft {
  Sensor leftSensor, rightSensor;
  float yaw, lastYaw;
} hovercraft;

bool turning = false;
float thresholdDist = 13.5;

uint16_t icr = 0xffff;


void initPVM() {
  DDRB  |= _BV(PB1) | _BV(PB2);       /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                 /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS11);                  /* prescaler 1 */
  ICR1 = icr;                         /* TOP counter value (freeing OCR1A*/
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

bool moveForward(){
  if (turning) return true;
  setServoAngle(90);
  delay(100);
  spinLiftFan();
  delay(100);
  spinThrustFan();
  return true;
}

void start(){
  moveForward();
}

bool stabilize(){
  if (turning) return true;
  setServoAngle(60);
  return true;
}

bool turn(){
  if (turning){
    setServoAngle(135);
  }
  return true;
}

bool stopTurn(){
  turning=false;
  moveForward();
  return false;
}

bool checkSensors(){
  if (turning) return true;
  updateSensorDistance(&hovercraft.leftSensor);
  updateSensorDistance(&hovercraft.rightSensor);
  if (hovercraft.rightSensor.distanceCM < thresholdDist){
    turning = true;
    turn();
  }
  return true;
}
bool getTurning(){
  return turning;
}
