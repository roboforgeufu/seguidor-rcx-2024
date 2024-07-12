#include <QTRSensors.h>

const int NUM_OF_SENSORS = 5;
const int EMITTER_PIN = 22;

QTRSensors line_follower_8left;
QTRSensors line_follower_8right;

uint16_t sensor_values_left[5];
uint16_t sensor_values_right[5];

void setup() {
  line_follower_8left.setTypeRC();
  line_follower_8right.setTypeRC();

  line_follower_8left.setEmitterPin(EMITTER_PIN);
  line_follower_8right.setEmitterPin(EMITTER_PIN);

  line_follower_8left.setSensorPins((const uint8_t[]){21,32,19,21,23}, NUM_OF_SENSORS);
  line_follower_8right.setSensorPins((const uint8_t[]){18,5,17,16,4}, NUM_OF_SENSORS);
  Serial.begin(115200);
}

void loop() {
  line_follower_8left.read(sensor_values_left);
  line_follower_8right.read(sensor_values_right);

  Serial.print("Esquerda: ");
  for(int i = 0; i <= NUM_OF_SENSORS; i++){
    Serial.print(sensor_values_left[i]);
    Serial.print(" ");
  }
  Serial.print("   ");

  Serial.print("Direita: ");
  for(int i = 0; i <= NUM_OF_SENSORS; i++){
    Serial.print(sensor_values_right[i]);
    Serial.print(" ");
  }
  Serial.println("");

  memset(sensor_values_left, 0, sizeof(sensor_values_left));
  memset(sensor_values_right, 0, sizeof(sensor_values_right));
  delay(500);
}








