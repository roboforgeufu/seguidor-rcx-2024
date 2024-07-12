#include <QTRSensors.h>

class DCMotor {
public:
  int voltage = 200;
  int in1;

  DCMotor(int in1) {
    in1 = in1;
    pinMode(in1, OUTPUT);
    analogWrite(in1, voltage);
  }

  void set_voltage(int volt_arg) {
    voltage = volt_arg;
  }

  void run() {
    analogWrite(in1, voltage);
  }

  void stop() {
    digitalWrite(in1, LOW);
  }
};

class LineFollower {
public:
  int ir1, ir2, ir3, ir4, ir5;
  int values[5];

  LineFollower(int port1, int port2, int port3, int port4, int port5) {
    ir1 = port1;
    ir2 = port2;
    ir3 = port3;
    ir4 = port4;
    ir5 = port5;
    pinMode(ir1, INPUT);
    pinMode(ir2, INPUT);
    pinMode(ir3, INPUT);
    pinMode(ir4, INPUT);
    pinMode(ir5, INPUT);
  }

  void refresh_values() {
    int ports[] = { ir1, ir2, ir3, ir4, ir5 };
    for (int i = 0; i <= 4; i++) {
      values[i] = !digitalRead(ports[i]);
    }
  }
};

DCMotor left_motor(2);
DCMotor right_motor(15);
LineFollower line_follower(13, 12, 14, 27, 33);

QTRSensors line_follower_8left;
QTRSensors line_follower_8right;

uint16_t sensor_values_left[5];
uint16_t sensor_values_right[5];

const int EN1 = 26;  // portas de controle PWM ponte H
const int EN2 = 25;
const int NUM_OF_SENSORS = 15;
const int EMITTER_PIN = 22;

double error;  // variaveis necessarias para controle pd
double previous_error;
double current_time;
double previous_time;
double elapsed_time;
double p_share;
double d_share;
double pd_correction;

const float KP = 4;  // constantes de calibracao
const float KD = 1.5;

const float SPEED = 120;
const float VOLT = 9;

float voltage = map(VOLT, 0, 12, 0, 255);
float left_speed;
float right_speed;
float left_volt;
float right_volt;

int objects_found[NUM_OF_SENSORS];
int current_object_info[2];
float sensor_dist_to_line[] = {-88, -81, -72, -66, -58, -42, -23, 0, 23, 42, 58, 66, 72, 81, 88};
int line_flag = 0;

bool all_sensor_reads[NUM_OF_SENSORS];

void setup() {
  Serial.begin(115200);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  Serial.print("Voltage: ");
  Serial.println(voltage);

  line_follower_8left.setTypeRC();
  line_follower_8right.setTypeRC();
  line_follower_8left.setEmitterPin(EMITTER_PIN);
  line_follower_8right.setEmitterPin(EMITTER_PIN);
  line_follower_8left.setSensorPins((const uint8_t[]){33, 27, 18, 19, 22}, 5);
  line_follower_8right.setSensorPins((const uint8_t[]){32, 5, 17, 16, 4}, 5);
  
  Serial.begin(115200);
}

void loop() {

  // monta o array com os valores lidos pelos sensores
  line_follower.refresh_values();  // atualiza os valores do sensor de 5 canais
  // TO DO: adicionar os valores dos sensores de 8 canais
  line_follower_8left.read(sensor_values_left);
  line_follower_8right.read(sensor_values_right);

  // Monta array de todos os sensores
  for (int i = 0; i <= NUM_OF_SENSORS - 1; i++) {
    bool read;
    if (i < 5){
      read = (bool) (sensor_values_left[i] >= 500);
    }else if (i < 10) {
      read = line_follower.values[i-5];
    } else {
      read = (bool) (sensor_values_right[i-10] >= 500);
    }
    all_sensor_reads[i] = read;
  }

  // print array
  Serial.println("Print Array Sensores:");
  for (int i = 0; i <= NUM_OF_SENSORS - 1; i++) {
    Serial.print(all_sensor_reads[i]);
      Serial.print(" ");
  }
  Serial.println("");

  int count_sensors = 0;
  int lines = 0;
  for(int i = 0; i <= NUM_OF_SENSORS - 1; i++){
    if (i >= 10 && line_follower.values[i] == 0) {
      count_sensors += 1;
    }
  }
  if(count_sensors >= 4){
    lines++;
    if (lines >= 2){
      while (elapsed_time <= 1){
        previous_time = current_time;
        current_time = millis();
        elapsed_time = (current_time - previous_time) / 1000;
      }
      while(1){
        left_motor.stop();
        right_motor.stop();
      }
    }
  }


  // // calcula o tamanho dos objetos lidos pelos sensores
  // int start_pos = 0;
  // int finish_pos = 0;
  // bool is_counting = false;
  // for (int i = 0; i <= NUM_OF_SENSORS - 1; i++) {
  //   if (line_follower.values[i] == 1) {
  //     if (is_counting == false){
  //       start_pos = i;
  //       is_counting = true;
  //     }
  //   }
  //   else {
  //     if (is_counting == true){
  //       finish_pos = i - 1;
  //       // if (start_pos == 0) {
  //       //   current_object_info[0] = start_pos;
  //       // }
  //       // else {
  //       //   current_object_info[0] = start_pos - 1;
  //       // }
  //       current_object_info[1] = (finish_pos - start_pos);
  //       Serial.print(current_object_info[0]);
  //       Serial.print(" ");
  //       Serial.print(current_object_info[1]);
  //       Serial.println("");
  //       for (int j = 0; j <= current_object_info[1]; j++) {
  //         objects_found[current_object_info[0]] += abs(sensor_dist_to_line[current_object_info[0] + j]);
  //       }
  //       memset(current_object_info, 0, sizeof(current_object_info));
  //       is_counting = false;
  //     }
  //   }
  // }

  // // print objetos
  // Serial.println("Print Objetos:");
  // for (int i = 0; i <= NUM_OF_SENSORS - 1; i++) {
  //   Serial.print(objects_found[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");

  // salva o erro anterior e calcula o novo erro
  previous_error = error;
  float count = 0;
  error = 0;
  for(int i = 5; i <= NUM_OF_SENSORS - 6; i++){
    if (line_follower.values[i] == 0) {
      error += sensor_dist_to_line[i];
      count += 1;
    }
  }
  if (count > 0){
    error = error/count;
  }
  else {
    error = previous_error;
  }

  // calculo do tempo passdo entre as iteracoes
  previous_time = current_time;  
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000;

  // calculo da correcao pd
  p_share = KP * error;
  d_share = KD * ((error - previous_error)/elapsed_time);
  pd_correction = p_share + d_share;
  pd_correction = constrain(pd_correction, -SPEED, SPEED);

  // print correcao pd
  Serial.println("Print Correcao:");
  Serial.print(error);
  Serial.print(" ");
  Serial.print(p_share);
  Serial.print(" ");
  Serial.print(d_share);
  Serial.print(" ");
  Serial.print(pd_correction);
  Serial.println("");

  // aplica a correcao PD nos motores
  int increment = 0;
  if (line_flag >= 2) {
    increment = 10 * (line_flag/2);
  }
  if (increment > 100) {
    increment = 120;
  }

  // motor esquerdo
  left_speed = SPEED + pd_correction + increment;
  analogWrite(EN1, left_speed);
  left_volt = voltage + (pd_correction/3);
  left_volt = constrain(left_volt, 123, 255); 
  left_motor.set_voltage(left_volt);
  left_motor.run();

  // motor direito
  right_speed = SPEED - pd_correction + increment;
  analogWrite(EN2, right_speed);
  right_volt = voltage - (pd_correction/3);
  right_volt = constrain(right_volt, 123, 255); 
  right_motor.set_voltage(right_volt);
  right_motor.run();

  if (left_speed == right_speed){
    line_flag++;
  }
  else {
    line_flag = 0;
  }

  // print motores
  Serial.println("Print Motores:");
  Serial.print(left_speed);
  Serial.print(" ");
  Serial.println(right_speed);

  // TO DO: evitar escapar da linha com os sensores mais externos

  // memset(objects_found, 0, sizeof(objects_found));

  delay(20);
}
