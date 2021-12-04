#include "IMU.h"
#include "PVMInterface.h"
#include "sensor.h"

unsigned long timer;

struct Hovercraft {
  Sensor leftSensor, rightSensor;
  float dist_x = 0, dist_y = 0;
  float vel_x = 0, vel_y = 0;
} hovercraft;


void setup() {
  initIMU();
  updateIMU();
  timer = millis();
}

void loop() {
  delta_t=getTimer(); //delta-t: time since last loop iteration
  resetTimer();
  updateIMU();

  //x-direction
  float accel_x = getAccelX(); //get x-acceleration in mm/s^2
  dist_x = dist_x + (vel_x * delta_t) + (accel_x * 10 * pow(delta_t, 2)); //calculate and update new x-distance
  vel_x += accel_x * 10 * delta_t; //update x-velocity (cm/s)

  //y-direction
  float accel_xy = getAccelY(); //get y-acceleration in mm/s^2
  dist_y = dist_y + (vel_y * delta_t) + (accel_y * 10 * pow(delta_t, 2)); //calculate and update new y-distance
  vel_y += accel_y * 10 * delta_t; //update y-velocity (cm/s)

}

unsigned long getTimer(){
  return (millis() - timer);
}

void resetTimer(){
  timer = millis();
}
