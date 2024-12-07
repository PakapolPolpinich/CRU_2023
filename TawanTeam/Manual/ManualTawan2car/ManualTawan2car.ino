#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

//// Multi-core
//TaskHandle_t Task1;
//
//volatile uint8_t data_valid_flag = 0;

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define STOP 0


#define PWMFreq 1000
#define PWMResolution 8

#define SERVO 22
#define GRIPPER 23
Servo servo;
Servo gripper;

struct MOTOR_PINS {
  uint8_t IN1;
  uint8_t IN2;
  uint8_t EN;
  uint8_t PWMSPEEDCHANNEL;
};

MOTOR_PINS motorPins[] = {
  { 21, 19, 18, 4 },  // LEFT_MOTOR
  { 25, 33, 32, 6 },  // RIGHT_MOTOR
};

void Motor_Init() {
  for (uint8_t i = 0; i < 2; ++i) {
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

void movement(bool LU, bool LD, bool RU, bool RD) {
  if (LU == 0) {
    rotateMotor(LEFT_MOTOR, 100);
  } else if (LD == 0) {
    rotateMotor(LEFT_MOTOR, -100);
  } else {
    rotateMotor(LEFT_MOTOR, STOP);
  }
  if (RU == 0) {
    rotateMotor(RIGHT_MOTOR, 100);
  } else if (RD == 0) {
    rotateMotor(RIGHT_MOTOR, -100);
  } else {
    rotateMotor(RIGHT_MOTOR, STOP);
  }
}

typedef struct struct_message {
  //  int pwmMotorLEFT;
  //  int pwmMotorRIGHT;
  uint8_t position;
  bool LbuttonUp;
  bool LbuttonDown;
  bool LbuttonLeft;
  bool LbuttonRight;
  bool RbuttonUp;
  bool RbuttonDown;
 
} struct_message;
struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //  if(data_valid_flag == 0)// Set flag when the flag is cleared
  //    data_valid_flag = 1;
}


void setup() {
  servo.attach(SERVO);
  gripper.attach(GRIPPER);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect(); 
  WiFi.setSleep(false);
  Motor_Init();
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //  Serial.println("Error initializing ESP-NOW");
    return;
  }

  //xTaskCreatePinnedToCore(
  //                    Task1code,   /* Task function. */
  //                    "Task1",     /* name of task. */
  //                    10000,       /* Stack size of task */
  //                    NULL,        /* parameter of the task */
  //                    1,           /* priority of the task */
  //                    &Task1,      /* Task handle to keep track of created task */
  //                    0);          /* pin task to core 0 */

  esp_now_register_recv_cb(OnDataRecv);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}


void action() {
  if (myData.position == 0 ) {
    servo.write(120);
  }
  if (myData.position > 79 ) { // test servo again
    servo.write(myData.position);
  }
  if (myData.LbuttonLeft == 0) {
    gripper.write(145);
  } else if (myData.LbuttonRight == 0) {
    gripper.write(175);
  }
 
  if (myData.LbuttonUp == 0 && myData.LbuttonDown == 0 && myData.RbuttonUp == 0 && myData.RbuttonDown == 0) {
    rotateMotor(LEFT_MOTOR, STOP);
    rotateMotor(RIGHT_MOTOR, STOP);
  } else {
    movement(myData.LbuttonUp, myData.LbuttonDown, myData.RbuttonUp, myData.RbuttonDown);
  }
  //  rotateMotor(LEFT_MOTOR,myData.pwmMotorLEFT);
  //  rotateMotor(RIGHT_MOTOR, myData.pwmMotorRIGHT);

  // rotateMotor(LEFT_MOTOR, -100);
  Serial.print(myData.LbuttonUp);
  Serial.print(myData.LbuttonDown);
  Serial.print(myData.RbuttonUp);
  Serial.print(myData.RbuttonDown );
  Serial.print(" position:");
  Serial.println(myData.position);


}
void loop() {
   action();
}

//void Task1code( void * pvParameters ){
//  while(1){
//    if(data_valid_flag == 1){
//      digitalWrite(2, HIGH);
//      action();
//      data_valid_flag = 0;// Clear flag
//      digitalWrite(2, LOW);
//    }
//
//  }
//
//}
