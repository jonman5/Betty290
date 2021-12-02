#include "IMU.h"
#include "PVMInterface.h"
#include "sensor.h"

struct Hovercraft {
  Sensor leftSensor, rightSensor;
  float yaw, lastYaw;
} hovercraft;

void setup() {
  Serial.begin(9600);
  hovercraft.leftSensor = createSensor(PD3);
  hovercraft.rightSensor = createSensor(PD4);
  initPVM();
  //initIMU();
  //delay(3000);
  //updateIMU();
  //hovercraft.yaw = 0;
}

void loop() {
  updateSensorDistance(&hovercraft.leftSensor);
  updateSensorDistance(&hovercraft.rightSensor);

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

  //Move forward
  setServoAngle(90);
  

  // spinLiftFan();
  // spinThrustFan();
  delay(500);

}
