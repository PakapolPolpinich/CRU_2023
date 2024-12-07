#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder;


//#define LEFT_JOY_Y 39
//#define RIGHT_JOY_X 35

int Motorleft = 0;
int Motorright = 0;


#define EN_2 16
#define EN_1 17

const uint8_t buttons[] = { 23, 13, 22, 5, 3, 19,};
bool states[] = { false, false, false, false, false, false,};
bool lastStates[] = { false, false, false, false, false, false};

uint8_t broadcastAddress[] = { 0xB0, 0xA7, 0x32, 0xD8, 0x91, 0x24 };

int counter = 0;
int currentStateCLK;
int previousStateCLK; 

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

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
   Serial.print("\r\nLast Packet Send Status:\t");
   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

//void JoyPWM(int joyy, int joyx) {
//  int y = map(joyy, 0, 4095, -2047, 2047);
//  int x = map(joyx, 0, 4095, -2047, 2047);
//  if (y > -130 && y < 0) {
//    y = 0;
//  }
//  if (x > -185 && x < 0) {
//    x = 0;
//  }
//  float m1 = (float)y + (-1 * x);
//  float m2 = (float)y + x;
//  Motorleft = (int)(m1 * 0.062286);
//  Motorright = (int)(m2 * 0.062286);
//}


void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }
  encoder.attachFullQuad(17, 16);
  encoder.setCount(120);

//  pinMode(LEFT_JOY_Y, INPUT);
//  pinMode(RIGHT_JOY_X, INPUT);

  WiFi.mode(WIFI_STA);
  //WiFi.disconnect();
  WiFi.setSleep(false);// add
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  //add

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
}


void loop() {

//  JoyPWM(analogRead(LEFT_JOY_Y), analogRead(RIGHT_JOY_X));
//  myData.pwmMotorLEFT = Motorleft;
//  myData.pwmMotorRIGHT = Motorright;

  int rotate = encoder.getCount();  // servo updown
  if (rotate < 80) {//85
    rotate = 80;
    encoder.setCount(80);
  } else if (rotate > 165) {//165
    rotate = 165;
    encoder.setCount(165);
  }
  myData.position = rotate;
  Serial.print(myData.position);


  for (uint8_t i = 0; i < 6; i++) {
    debounce(i);
  }

  myData.LbuttonUp = states[0];  //
  myData.LbuttonDown = states[1];
  myData.LbuttonLeft = states[2];   // gripper
  myData.LbuttonRight = states[3];  //gripper
  myData.RbuttonUp = states[4];
  myData.RbuttonDown = states[5];
 
  Serial.print(myData.LbuttonUp);
  Serial.print(myData.LbuttonDown);
  Serial.print(myData.LbuttonLeft);
  Serial.print(myData.LbuttonRight);
  Serial.print(myData.RbuttonUp);
  Serial.println(myData.RbuttonDown);
  //  delay(1);
  //// // Serial.print("x");
  //Serial.println(digitalRead(1))
  //Serial.print(analogRead(LEFT_JOY_X));
  //Serial.print(" ");
  //Serial.print(analogRead(LEFT_JOY_Y));
  //Serial.print(" ");
  //Serial.print(analogRead(RIGHT_JOY_X));
  //Serial.print(" ");
  //Serial.println(analogRead(RIGHT_JOY_Y));
//  Serial.print(" ");
//  Serial.print(myData.pwmMotorLEFT);
//  Serial.print(" ");
//  Serial.print(myData.pwmMotorRIGHT);
//  Serial.println(" ");
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

   if (result == ESP_OK) {
    // Serial.println("Sent with success");
   }
   else {
    // Serial.println("Error sending the data");
   }
}
