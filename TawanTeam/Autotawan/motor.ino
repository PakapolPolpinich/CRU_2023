#define PWMFreq 5000
#define PWMResolution 12
#define STOP 0

struct MOTOR_PINS {
  uint8_t IN;
  uint8_t PWMSPEEDCHANNEL;
};

MOTOR_PINS motorPins[] = {
  { 32, 12 },  // FRONT_LEFT_MOTOR IN1
  { 33, 13 },  // FRONT_LEFT_MOTOR IN2
  { 18, 10 },  // FRONT_RIGHT_MOTOR IN1
  {  5, 11 },  // FRONT_RIGHT_MOTOR IN2
  { 25, 4 },  // BACK_LEFT_MOTOR IN1
  { 26, 5 },  // BACK_LEFT_MOTOR IN2
  { 16, 6 },  // BACK_RIGHT_MOTOR IN1
  { 17, 7 },  // BACK_RIGHT_MOTOR IN2
};

void Motor_Init() {
  for (uint8_t i = 0; i < 8; ++i) {
    pinMode(motorPins[i].IN, OUTPUT);
    ledcSetup(motorPins[i].PWMSPEEDCHANNEL, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].IN, motorPins[i].PWMSPEEDCHANNEL);
  }
  for (uint8_t i = 0; i < 8; i += 2) {
    rotateMotor(i, STOP);
  }
}

void rotateMotor(uint8_t motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, 0);
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, abs(motorSpeed));
  } else if (motorSpeed > 0) {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, abs(motorSpeed));
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, 0);
  } else {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, 0);
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, 0);
  }
}
