#define THRUST_FAN_PIN PD6
#define LIFT_FAN_PIN OCR1B
#define SERVO_PIN OCR1A
#include "sensor.h"


void initPVM();
void spinLiftFan();
void setServoAngle(int angle);
void setServoPin(int value);
void spinThrustFan();
void start();
bool moveForward();
bool stabilize();
bool turn();
bool checkSensors();
bool stopTurn();
bool getTurning();
