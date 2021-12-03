#include "Arduino.h"
#include "PVMInterface.h"
#include "arduino-timer.h"
struct Hovercraft {
  Sensor leftSensor, rightSensor;
  float yaw, lastYaw;
} hovercraft;

bool stabilizing = false;
bool turning = false;
float thresholdDist = 25;

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
  Serial.println("Moving forward now.");
  //delay(100);
  //spinLiftFan();
  //delay(100);
  //spinThrustFan();
  return true;
}

void start(){
  moveForward();
}

bool stabilize(){
  if (turning) return true;
  setServoAngle(110);
  stabilizing = true;
  Serial.println("Stabilizing now.");
  return true;


}

void turnRight(){
  if (turning){
    setServoAngle(150);
  }
  delay(4000);
  stopTurn();
}

void turnLeft(){
  if (turning){
    setServoAngle(30);
  }
  delay(4000);
  stopTurn();
}

void stopTurn(){
  turning=false;
  moveForward();
}

bool checkSensors(bool turnRight){
  updateSensorDistance(&hovercraft.leftSensor);
  updateSensorDistance(&hovercraft.rightSensor);
  if (turnRight && (hovercraft.rightSensor.distanceCM > thresholdDist)){
    //if next turn is right and right sensor does not see a wall within thresholdDist, turn right
    turning = true;
    Serial.println("Turning");
    turnRight();
    return true;
  }
  else if (!turnRight && (hovercraft.leftSensor.distanceCM > thresholdDist)){
    //if next turn is not right and left sensor does not see a wall within thresholdDist, turn left
    turning = true;
    Serial.println("Turning");
    turnLeft();
    return true;
  }
  return false; //no turn was executed, return false
}
bool getTurning(){
  return turning;
}
