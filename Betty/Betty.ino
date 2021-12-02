#include "IMU.h"
#include "PVMInterface.h"
#include "arduino-timer.h"

auto timer = timer_create_default(); //create a timer instance *timer*

void setup() {
  Serial.begin(9600);
  // hovercraft.leftSensor = createSensor(PD3);
  // hovercraft.rightSensor = createSensor(PD4);
  initPVM();
  //thresholdDist = 13.5; //Threshold distance in cm that defines when to start turning
  //initIMU();
  //delay(3000);
  //updateIMU();
  //hovercraft.yaw = 0;
}

void loop() {
  // updateSensorDistance(&hovercraft.leftSensor);
  // updateSensorDistance(&hovercraft.rightSensor);

  // updateIMU();
  // float currentYaw = getGyroYaw() + 180.0;
  // float deltaYaw = currentYaw - hovercraft.lastYaw;
  // float inverseDeltaYaw = 360.0 - deltaYaw;
  // hovercraft.yaw += deltaYaw < inverseDeltaYaw ? deltaYaw : inverseDeltaYaw;
  // Serial.print("Yaw: ");
  // Serial.println(hovercraft.yaw);
  // hovercraft.lastYaw = currentYaw;

  // Serial.print("Left Sensor: ");
  // Serial.print(hovercraft.leftSensor.distanceCM);
  // Serial.println(" (CM)");
  // Serial.print("Right Sensor: ");
  // Serial.print(hovercraft.rightSensor.distanceCM);
  // Serial.println(" (CM)");


  timer.tick();
  //Move forward at 50ms
  timer.at(50, start);
  timer.every(2500, stabilize); //stabilize every 2500ms
  timer.every(3250, moveForward); //go back to moving forward every 3250ms
  timer.every(100, checkSensors); //check sensors every 100ms
  if (getTurning()){
    timer.in(2500, stopTurn);
  }



  // spinLiftFan();
  // spinThrustFan();
  delay(500);

}
