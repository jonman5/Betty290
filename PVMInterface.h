#define THRUST_FAN_PIN PD6
#define LIFT_FAN_PIN OCR1B
#define SERVO_PIN OCR1A

void initPVM();
void spinLiftFan();
void setServoAngle(float angle);
void spinThrustFan(int speed);
