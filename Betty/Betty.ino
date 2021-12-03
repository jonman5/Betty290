#include "IMU.h"
#include "PVMInterface.h"
#include "arduino-timer.h"

auto timer0 = timer_create_default(); //create a timer instance *timer*
auto timer1 = timer_create_default(); //create a timer instance *timer*
auto timer2 = timer_create_default(); //create a timer instance *timer*
unsigned long timer;
bool init_ = false;
bool stab = true;


void setup() {
  Serial.begin(9600);
  // hovercraft.leftSensor = createSensor(PD3);
  // hovercraft.rightSensor = createSensor(PD4);
  initPVM();
  timer = millis();
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
  // delay(500);



  unsigned long currentTime = millis();
  unsigned long time_elapsed = currentTime - timer;
  unsigned long time_change = 2000;
  if (init_){
    moveForward();
    stab=false;
  }
  if (time_elapsed >=time_change){
    
    if (!stab){
      stabilize();
      stab=true;
      time_change = 500; //stabilize for 1000ms 
    }
    else{
      moveForward();
      stab=false;
      time_change = 1000; //move forward for 3000ms 
    }
    timer = millis();
  }
  checkSensors();
  
  //Move forward at 50ms
  //timer1.in(500, start);
  //timer1.every(30000, stabilize); //stabilize every 
  //timer1.every(20000, moveForward); //stabilize every 

  //timer.every(100, checkSensors); //check sensors every 100ms
  //if (getTurning()){
  //  timer.in(2500, stopTurn);
  //}



  spinLiftFan();
  spinThrustFan();

}
