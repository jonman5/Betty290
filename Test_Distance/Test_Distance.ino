#include "IMU.h"
#include "PVMInterface.h"
//#include "sensor.h"

unsigned long timer;

struct Hovercraft {
  Sensor leftSensor, rightSensor;
  unsigned long dist_x = 0, dist_y = 0;
  float vel_x = 0, vel_y = 0;
  float accel_x = 0, accel_y = 0;
} hovercraft;


void setup() {
  initIMU();
  updateIMU();
  timer = millis();
}

void loop() {
  unsigned long delta_t = getTimer(); //delta-t: time since last loop iteration
  resetTimer();
  updateIMU();

  //x-direction
  hovercraft.accel_x = getAccelX() / 10; //get x-acceleration in cm/s^2
  (hovercraft.accel_x < 1) ? hovercraft.accel_x = 0: hovercraft.accel_x = hovercraft.accel_x; 
  hovercraft.dist_x = hovercraft.dist_x + (hovercraft.vel_x * delta_t) + (hovercraft.accel_x * pow(delta_t, 2)); //calculate and update new x-distance
  hovercraft.vel_x += hovercraft.accel_x * delta_t; //update x-velocity (cm/s)

  //y-direction
  // float accel_xy = getAccelY(); //get y-acceleration in mm/s^2
  // dist_y = dist_y + (vel_y * delta_t) + (accel_y * 10 * pow(delta_t, 2)); //calculate and update new y-distance
  // vel_y += accel_y * 10 * delta_t; //update y-velocity (cm/s)
  Serial.print("Velocity in x-direction: ");
  Serial.println(hovercraft.vel_x);
  Serial.print("Distance in x-direction: ");
  Serial.println(hovercraft.dist_x);

}

unsigned long getTimer(){
  return (millis() - timer);
}

void resetTimer(){
  timer = millis();
}
