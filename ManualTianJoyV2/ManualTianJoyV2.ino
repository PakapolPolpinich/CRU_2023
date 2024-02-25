#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;
ESP32Encoder encoder2;

#define LEFT_JOY_Y 34
#define RIGHT_JOY_X 36
#define EN_2 16
#define EN_1 17

int Motorleft = 0;
int Motorright = 0;

const uint8_t buttons[] = { 23, 13, 22, 5, 3, 19, 18, 21 };
bool states[] = { false, false, false, false, false, false, false, false };
bool lastStates[] = { false, false, false, false, false, false, false, false };

uint8_t broadcastAddress[] = { 0xB0, 0xA7, 0x32, 0xDB, 0xB6, 0x6C };

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

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool debounce(uint8_t i) {
  static unsigned long lastTime[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  bool currentState = digitalRead(buttons[i]);

  if (currentState != lastStates[i]) {
    lastTime[i] = millis();
    lastStates[i] = currentState;
  }

  if (millis() - lastTime[i] > 40) {
    states[i] = currentState;
  }

  return states[i];
}
void JoyPWM(int joyy, int joyx) {
  int y = map(joyy, 0, 4095, -2047, 2047);
  int x = map(joyx, 0, 4095, -2047, 2047);
  if (y > -155 && y < 0) {
    y = 0;
  }
  if (x > -160 && x < 0) {
    x = 0;
  }
  float m1 = (float)y + (-1 * x);
  float m2 = (float)y + x;
  Motorleft = (int)(m1 * 0.062286);
  Motorright = (int)(m2 * 0.062286);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_JOY_Y, INPUT);
  pinMode(RIGHT_JOY_X, INPUT);
  encoder.attachFullQuad(17, 16);
  encoder.setCount(80);
  encoder2.attachFullQuad(33, 32);
  encoder2.setCount(0);

  for (uint8_t i = 0; i < 8; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
  // Serial.print("y:");
  // Serial.print(map(analogRead(34), 0, 4095, -2047, 2047));
  // Serial.print(" x:");
  // Serial.println(map(analogRead(36), 0, 4095, -2047, 2047));
   JoyPWM(analogRead(LEFT_JOY_Y), analogRead(RIGHT_JOY_X));
   myData.pwmMotorLEFT = Motorleft;
   myData.pwmMotorRIGHT = Motorright;

  int counter = encoder2.getCount();
  uint8_t rotate = encoder.getCount();
  if (rotate < 90) {
    rotate = 90;
    encoder.setCount(90);
  } else if (rotate > 165) {
    rotate = 165;
    encoder.setCount(165);
  }
  myData.position = rotate;
  if (counter <= 0) {
    counter = 0;
    encoder2.setCount(0);
  } else if (counter >45 ) {
    counter = 45;
    encoder2.setCount(45);
  }
  myData.gripper = counter;
  for (uint8_t i = 0; i < 8; i++) {
    debounce(i);
  }

 // myData.LbuttonUp = states[0];
 // myData.LbuttonDown = states[1];
  myData.LbuttonLeft = states[2];
  myData.LbuttonRight = states[3];
 // myData.RbuttonUp = states[4];
 // myData.RbuttonDown = states[5];
  myData.RbuttonLeft = states[6];
  myData.RbuttonRight = states[7];

  //Serial.print(Data.LbuttonUp);
  //Serial.print(Data.LbuttonDown);
  // Serial.print(myData.LbuttonLeft);
  // Serial.print(myData.LbuttonRight);
  //Serial.print(Data.RbuttonUp);
  //Serial.print(Data.RbuttonDown );
  //Serial.print(Data.RbuttonLeft);
  //Serial.println(Data.RbuttonRight);
  //  delay(1);
  //// // Serial.print("x");
  //Serial.println(digitalRead(1))
  // Serial.print(" ");
  // Serial.print(myData.pwmMotorLEFT);
  // Serial.print(" ");
  // Serial.print(myData.gripper);
  // Serial.print(" ");
  // Serial.println(myData.position);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

//   if (result == ESP_OK) {
//   Serial.println("Sent with success");
//   } else {
//    Serial.println("Error sending the data");
//   }
}
