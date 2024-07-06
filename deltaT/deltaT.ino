class DCMotor {
public:
  int speed = 200;
  int in1;

  DCMotor(int in1) {
    in1 = in1;
    pinMode(in1, OUTPUT);
    analogWrite(in1, speed);
  }

  void set_speed(int spd) {
    speed = spd;
  }

  void run() {
    analogWrite(in1, speed);
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

const int EN1 = 26;
const int EN2 = 25;

void setup() {

  Serial.begin(115200);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
}

void loop() {

  while (1) {

    int vel_left = 80;
    int vel_right = 80;

    line_follower.refresh_values();

    for (int i = 0; i <= 4; i++) {
      Serial.print(line_follower.values[i]);
    }
    Serial.println("");

    // Sensores intermediários
    if (line_follower.values[3] == 1) {
      vel_left = 120;
      vel_right = 40;
    }
    if (line_follower.values[1] == 1) {
      vel_right = 120;
      vel_left = 40;
    }

    if (line_follower.values[0] == 1 && line_follower.values[4] == 1) {
      // cruzamento (tratativa pra não travar)
      vel_left = 80;
      vel_right = 80;
    }
    
    // Sensores das extremidades
    else if (line_follower.values[4] == 1) {
      while(line_follower.values[2] != 1) {
        line_follower.refresh_values();
        vel_left = 160;
        vel_right = 0;
        analogWrite(EN1, vel_left);
        left_motor.set_speed(100);
        left_motor.run();
        analogWrite(EN2, vel_right);
        right_motor.set_speed(100);
        right_motor.run();
      }
    }
    else if (line_follower.values[0] == 1) {
      while(line_follower.values[2] != 1) {
        line_follower.refresh_values();
        vel_left = 0;
        vel_right = 160;
        analogWrite(EN1, vel_left);
        left_motor.set_speed(100);
        left_motor.run();
        analogWrite(EN2, vel_right);
        right_motor.set_speed(100);
        right_motor.run();
      }
    }

    
    Serial.print("Esquerda: ");
    Serial.print(vel_left);

    Serial.print("\tDireita: ");
    Serial.println(vel_right);

    analogWrite(EN1, vel_left);
    left_motor.set_speed(200);
    left_motor.run();

    analogWrite(EN2, vel_right);
    right_motor.set_speed(200);
    right_motor.run();
  }
}