#include "PVMInterface.h"
#include "sensor.h"

unsigned long timer;

struct Hovercraft {
  Sensor frontSensor, backSensor;
  int counter;
} hovercraft;

enum State {
  X_MOVE,
  Y_MOVE
} state = X_MOVE;

enum Direction {
  FRONT = 0,
  BACK = 1
} direction = FRONT;

void setup() {
  Serial.begin(9600);
  hovercraft.frontSensor = createSensor(PD3);
  hovercraft.backSensor = createSensor(PD4);
  initPVM();
  spinThrustFan(200);
  spinLiftFan();
    hovercraft.counter = 0;
}



void loop() {
  //hovercraft.counter += 1;
  updateSensorDistance(&hovercraft.frontSensor);
  updateSensorDistance(&hovercraft.backSensor);
  // Serial.print("Left Sensor: ");
  // Serial.print(hovercraft.leftSensor.distanceCM);
  // Serial.println(" (CM)");
  // Serial.print("Right Sensor: ");
  // Serial.print(hovercraft.rightSensor.distanceCM);
  // Serial.println(" (CM)");

 
  //spinThrustFan();
  // Serial.print("Angle : ");
  // float a = 90.0f + 40.0f * (cos((float) hovercraft.counter / 20.0f) - 1.0f) / 2.0f;
  // float a = 70.0f;
  // Serial.println(a);
  // setServoAngle(a);



  switch(state) {
    case X_MOVE:
      if(direction == FRONT) {
        setServoAngle(25);
        if(hovercraft.frontSensor.distanceCM <= 16) {
          spinThrustFan(195);
          state = Y_MOVE;
          //hovercraft.counter = 0;
          resetTimer();
        }
        else if(hovercraft.frontSensor.distanceCM <= 65) {
          spinThrustFan(175);
        }
        else {
          spinThrustFan(200);
        }
      } 
      else {
        setServoAngle(155);
        if(hovercraft.backSensor.distanceCM <= 16) {
          spinThrustFan(195);
          state = Y_MOVE;
          // hovercraft.counter = 0;
          resetTimer();
        }
        else if(hovercraft.backSensor.distanceCM <= 65) {
          spinThrustFan(175);
        }
        else {
          spinThrustFan(200);
        }
      }
      break;
    case Y_MOVE:
      if(direction == FRONT) {
        setServoAngle(60);
      } else {
        setServoAngle(120); 
      }
      if (getTimer() > 2000) {
        state = X_MOVE;
        direction = (direction == FRONT) ? BACK : FRONT;
      }
      break;
  }
}

unsigned long getTimer(){
  return (millis() - timer);
}

void resetTimer(){
  timer = millis();
}
