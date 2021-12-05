#include "Arduino.h"
#include "sensor.h"

void swap(unsigned long *a, unsigned long *b);
void bubbleSort(unsigned long *buffer, int length) {
  int i, j;
  for(int i = 0; i < length - 1; i += 1) {
    for(int j = 0; j < length - i - 1; j += 1) {
      if(buffer[j] > buffer[j + 1]) swap(&buffer[j], &buffer[j + 1]);
    }
  }
}
int getAverage(unsigned long *buffer, int length) {
  unsigned long sum = 0;
  for(int i = 0; i < length; i += 1) {
    sum += buffer[i];
  }
  return sum / length;
}
void swap(unsigned long *a, unsigned long *b) {
  unsigned long t = *a;
  *a = *b;
  *b = t;
}

Sensor createSensor(int pinID) {
  Sensor sensor;
  sensor.pinID = pinID;
  pinMode(pinID, INPUT);
  return sensor;
}
void updateSensorDistance(Sensor *sensor) {
  int const BUFFER_SIZE = 8;
  unsigned long buffer[BUFFER_SIZE] = {0};
  //Get a set of samples.
  for(int i = 0; i < BUFFER_SIZE; i += 1) {
    unsigned long const t = pulseIn(sensor->pinID, HIGH);
    buffer[i] = t / 58;
    delay(15);
  }

  //Order the samples
  bubbleSort(buffer, BUFFER_SIZE);
  int const BUFFER_CUTOUT = BUFFER_SIZE / 4;
  //Discard the lowest and highest samples and
  //get the average of the rest.
  unsigned long d = getAverage(&buffer[BUFFER_CUTOUT], BUFFER_SIZE - BUFFER_CUTOUT * 2);
  sensor->distanceCM = d;
  //sensor->distanceCM = pulseIn(sensor->pinID, HIGH) / 58;
}
