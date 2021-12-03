#include "IMU.h"
#include "PVMInterface.h"
#include "arduino-timer.h"

auto timer0 = timer_create_default(); //create a timer instance *timer*
auto timer1 = timer_create_default(); //create a timer instance *timer*
auto timer2 = timer_create_default(); //create a timer instance *timer*
unsigned long timer;
bool stab = false;
bool nextTurnRight = true;

void setup() {
  Serial.begin(9600);
  initPVM();
  timer = millis();
  moveForward();
  //thresholdDist = 13.5; //Threshold distance in cm that defines when to start turning
  //initIMU();
  //delay(3000);
  //updateIMU();
  //hovercraft.yaw = 0;
  // hovercraft.leftSensor = createSensor(PD3);
  // hovercraft.rightSensor = createSensor(PD4);
}

void loop() {
  unsigned long time_change = 1000;
  if (getTimer() >= time_change){

    if (!stab){
      stabilize();
      stab=true;
      time_change = 250; //stabilize for 250ms
    }
    else{
      moveForward();
      stab=false;
      time_change = 1000; //move forward for 1000ms
    }
    resetTimer();
  }
  if (checkSensors(nextTurnRight)){ //checks sensors, and checks if turn was executed
    nextTurnRight = (!nextTurnRight); //flip next turn direction
    resetTimer();
  }

  spinLiftFan();
  spinThrustFan();
//  delay(30);



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
// delay(500);
}


unsigned long getTimer(){
  return (millis() - timer);
}

void resetTimer(){
  timer = millis();
}
