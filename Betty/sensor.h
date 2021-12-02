typedef struct Sensor {
  int pinID;
  unsigned long distanceCM;
} Sensor;

Sensor createSensor(int pinID);
void updateSensorDistance(Sensor *sensor);
