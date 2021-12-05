#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

// Pin definitions
const int intPin = 13;  // These can be changed, 2 and 3 are the Arduinos ext int pins
const int myLed  = 12;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

void initIMU();
void updateIMU();
float getGyroYaw();
float getAccelX();
float getAccelY();
