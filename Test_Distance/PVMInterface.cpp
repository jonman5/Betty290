#include "Arduino.h"
#include "PVMInterface.h"


bool stabilizing = false;
bool turning = false;
float thresholdSideDist = 70;
float thresholdFrontDist = 70;

uint16_t icr = 0xffff;


void initPVM() {
  DDRB  |= _BV(PB1) | _BV(PB2);       /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                 /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS11);                  /* prescaler 1 */
  ICR1 = icr;                         /* TOP counter value (freeing OCR1A*/
  //hovercraft.leftSensor = createSensor(PD3);
  //hovercraft.rightSensor = createSensor(PD4);
}
void spinThrustFan(int speed) { //Speed is 0-255
  analogWrite(THRUST_FAN_PIN, speed);
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
  spinThrustFan(195);
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
    setServoAngle(10);
  }
  delay(1500);
  stopTurn();
}

void turnLeft(){
  if (turning){
    setServoAngle(170);
  }
  delay(4000);
  stopTurn();
}

void stopTurn(){
  turning=false;
  moveForward();
}

bool checkSensors(bool turnToRight){
  // updateSensorDistance(&hovercraft.leftSensor);
  // updateSensorDistance(&hovercraft.rightSensor);
  // // Serial.print("Left Sensor: ");
  // // Serial.print(hovercraft.leftSensor.distanceCM);
  // // Serial.println(" (CM)");
  // // Serial.print("Right Sensor: ");
  // // Serial.print(hovercraft.rightSensor.distanceCM);
  // // Serial.println(" (CM)");
  // // delay(1000);
  // if ( (hovercraft.leftSensor.distanceCM < thresholdFrontDist )){
  //   //if next turn is right and right sensor does not see a wall within thresholdDist, turn right
  //   if ((hovercraft.leftSensor.distanceCM > 180)){
  //     return false;
  //   }
  //   turning = true;
  //   Serial.println("Turning Right");
  //   spinThrustFan(165); //reduce thrust speed before turn
  //   delay(1000);
  //   turnRight();
  //   setServoAngle(90);
  //   delay (800);
  //   turnRight();
  //   return true;
  // }
  // else if (!turnToRight && (hovercraft.leftSensor.distanceCM > thresholdDist)){
  //   //if next turn is not right and left sensor does not see a wall within thresholdDist, turn left
  //   turning = true;
  //   Serial.println("Turning Left");
  //   spinThrustFan(150);
  //   turnLeft();
  //   return true;
  // }
  return false; //no turn was executed, return false
}
bool getTurning(){
  return turning;
}
