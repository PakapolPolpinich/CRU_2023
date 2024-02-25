#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>


#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 1
#define BACK_LEFT_MOTOR 2
#define BACK_RIGHT_MOTOR 3
#define STOP 0


#define PWMFreq 1000
#define PWMResolution 8

#define GRIPPER 23
#define SERVO 22
Servo servo;
Servo gripper;

uint8_t counter = 0;

bool n =0;

struct MOTOR_PINS {
  uint8_t IN1;
  uint8_t IN2;
  uint8_t EN;
  uint8_t PWMSPEEDCHANNEL;
};

MOTOR_PINS motorPins[4] = {
  { 18, 19, 21, 4 },  // FRONT_LEFT_MOTOR
  { 4, 2, 15, 5 },    // FRONT_RIGHT_MOTOR
  { 25, 33, 32, 6 },  // BACK_LEFT_MOTOR
  { 26, 27, 14, 7 }   // BACK_RIGHT_MOTOR
};

void Motor_Init() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motorPins[i].IN1, OUTPUT);
    pinMode(motorPins[i].IN2, OUTPUT);
    pinMode(motorPins[i].EN, OUTPUT);

    ledcSetup(motorPins[i].PWMSPEEDCHANNEL, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].EN, motorPins[i].PWMSPEEDCHANNEL);
    rotateMotor(i, STOP);
  }
}

void rotateMotor(uint8_t motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].IN1, HIGH);
    digitalWrite(motorPins[motorNumber].IN2, LOW);
  } else if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].IN1, LOW);
    digitalWrite(motorPins[motorNumber].IN2, HIGH);
  } else {
    digitalWrite(motorPins[motorNumber].IN1, LOW);
    digitalWrite(motorPins[motorNumber].IN2, LOW);
  }
  ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, abs(motorSpeed));
}

typedef struct struct_message {
  int pwmMotorLEFT;
  int pwmMotorRIGHT;
  //bool LbuttonUp;
  //bool LbuttonDown;
  bool LbuttonLeft;
  bool LbuttonRight;
  //bool RbuttonUp;
  //bool RbuttonDown;
  bool RbuttonLeft;
  bool RbuttonRight;
  uint8_t position;
  uint8_t gripper;
} struct_message;
struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}


void setup() {
  servo.attach(SERVO);
  gripper.attach(GRIPPER);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Motor_Init();
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  esp_now_register_recv_cb(OnDataRecv);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}


void loop() {

  if (myData.position == 0) {
    servo.write(120);
  }
  if (myData.position > 89) {
    servo.write(myData.position);
  }
   if (myData.LbuttonLeft == 0 && myData.LbuttonRight == 0) {
        gripper.write(10);
      } else {
        if (myData.LbuttonLeft == 0) {
          gripper.write(10 );
        } else if (myData.LbuttonRight == 0) {
          gripper.write(45);
        }    
      }
     
  Serial.println(n);
      rotateMotor(FRONT_LEFT_MOTOR, myData.pwmMotorLEFT);
      rotateMotor(FRONT_RIGHT_MOTOR, myData.pwmMotorRIGHT);
      rotateMotor(BACK_LEFT_MOTOR, myData.pwmMotorLEFT);
      rotateMotor(BACK_RIGHT_MOTOR, myData.pwmMotorRIGHT);
  }
