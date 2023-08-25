//Biblioteca do controle de PS4
#include <PS4Controller.h>



//pin definitions for Placa V2
#define PWMA 32
#define PWMB 13
#define A1  25
#define A2  33
#define B1  26
#define B2  27
#define deadzone 15

#define PS4_ADDRESS "84:15:51:7B:D0:35"


float v_l, v_a;
float last_error = 0;
float error_sum = 0;

int PS4R = 0;
int PS4L = 0;

int robot_id = 3;
int id;

float kps[3] = {2.0317307, 1.2771797, 1.702362};

float wheelRadius = 0.035/2;
float robotRadius = 0.075/2;
float ks = 0.99;
float kv = 0.1736;
float speedMin = 0.05;


void motor_R(float speedR) { // se o valor for positivo gira para um lado e se for negativo troca o sentido
  if (speedR > 0) {
    digitalWrite(A1, 1);
    digitalWrite(A2, 0);
  } else {
    digitalWrite(A1, 0);
    digitalWrite(A2, 1);
  }

  speedR = abs(speedR);
  Serial.print("R: ");
  float angular = speedR/wheelRadius;
  float voltage = ks + kv*angular;
  Serial.println(voltage);
  float pwm = map(voltage, 0, get_voltage(), 0, 255);
  if(pwm > 255) pwm = 255;
  if(speedR < speedMin) pwm = 0;
  ledcWrite(1, pwm);
}

void motor_L(float speedL) {
  if (speedL > 0) {
    digitalWrite(B1, 1);
    digitalWrite(B2, 0);
  } else {
    digitalWrite(B1, 0);
    digitalWrite(B2, 1);
  }

  speedL = abs(speedL);
  Serial.print("L: ");
  float angular = speedL/wheelRadius;
  float voltage = ks + kv*angular;
  Serial.print(voltage);
  Serial.print("\t");
  float pwm = map(voltage, 0, get_voltage(), 0, 255);
  if(pwm > 255) pwm = 255;
  if(speedL < speedMin) pwm = 0;
  ledcWrite(2, pwm);
}


void motors_control(float linear, float angular) {
  angular = angular + pid(angular, - get_theta_speed());
  float Vel_R = linear - robotRadius * angular; //ao somar o angular com linear em cada motor conseguimos a ideia de direcao do robo
  float Vel_L = linear + robotRadius * angular;

  motor_L(Vel_L);
  motor_R(Vel_R); //manda para a funcao motor um valor de -255 a 255, o sinal signifca a direcao
}


void setup(void) {
  Serial.begin(115200);

  // configuração de pinos
  PS4.begin(PS4_ADDRESS);
  Serial.println("Ready");
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);

  ledcSetup(1, 80000, 8);
  ledcSetup(2, 80000, 8);

  //pinMode(stby, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  //digitalWrite(stby, 1);
  digitalWrite(A1, 0);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 0);

  mpu_init();

 // ws2812_init();

  while(PS4.isConnected()!= true){
    delay(20);
  }
}


void loop() {
  while(PS4.isConnected()) {
      PS4L = PS4.LStickY();
      PS4R = PS4.RStickX();
      Serial.print("AnalogL: ");
      Serial.print(PS4L);
      Serial.print("\t");
      Serial.print("AnalogR: ");
      Serial.print(PS4R);
      Serial.print("\t");
      motors_control(PS4L/250,PS4R/40);
  }
    
  //Failsafe
  if(PS4.isConnected()!= true){
  motors_control(0,0);
  Serial.println("Restart");
  PS4.end();
  ESP.restart();
  delay(20);
  }
}

