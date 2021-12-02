#define THRUST_FAN_PIN PD6
#define LIFT_FAN_PIN OCR1B
#define SERVO_PIN OCR1A

struct Hovercraft {
  Sensor leftSensor, rightSensor;
  float yaw, lastYaw;
} hovercraft;

bool turning = false;
float thresholdDist = 0;

void initPVM();
void spinLiftFan();
void setServoAngle(int angle);
void setServoPin(int value);
void spinThrustFan();
void moveForward();
void stabilize();
void turn();
void checkSensors();
void stopTurn();
