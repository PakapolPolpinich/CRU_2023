#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 2
#define BACK_LEFT_MOTOR 4
#define BACK_RIGHT_MOTOR 6
#define STOP 0

#define MAX_VRPM 7040

#define limMaxInt 1384886.25
#define limMinInt -1403292.50

#include <WiFi.h>

const char* ssid = "Robotclub_KMITL_E12";
const char* password = "Kakorsetyor2022";
//IPAddress staticIP(192, 168, 1, 186);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 0, 0);
//IPAddress dns(192, 168, 1, 1);
char message[128];
char *ptr;

WiFiServer wifiServer(80);


long prevtime = 0;
long prevtime2 = 0;
int speedPWM[4]={0,0,0,0};//get value for naming pc
bool state =0;


struct PIDControl {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  bool state;
};

PIDControl PIDControllers[] = {
  {1.0, 0.005, 2.08, 0, 0,0},// PID for Front Left Motor 
  {0.5, 0.005, 2.08, 0, 0,0},// PID for Front Right Motor
  {1.0, 0.005, 2.5, 0, 0,0},// PID for Back Left Motor
  {0.5, 0.005, 2.08, 0, 0,0},// PID for Back Right Motor
};
struct Velocity {
  float Pulse;
  float PrevPulse;
  float Vrpm;
};

struct Velocity Count[] = {
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

float ChangetoRPM(long Pulse, uint8_t i) {
  float Vrpm = (((float)Pulse) - Count[i].PrevPulse) * 4.6875;// 1output/จำนวนcount*60s/ms*1/deltatime(100) = vrpm
  Count[i].PrevPulse = (float)Pulse;
  return Vrpm;
}

long PIDMOTOR(float setpoint, float rpm, float deltaTime, PIDControl &pid) {

  if(setpoint > 0 && pid.state == 1){
    pid.integral = 0;
//    Serial.print("first  ");
//    Serial.print(pid.integral);
//    Serial.print(" ");
    pid.state = 0;
  }else if (setpoint < 0 && pid.state == 0){
    pid.integral = 0;
//    Serial.print("second  ");
//    Serial.print(pid.integral);
//    Serial.print(" ");
    pid.state = 1;
  }
  
  float error = setpoint - rpm;
  pid.integral += (error * deltaTime);

  // Anti-windup
  if (pid.integral > limMaxInt) {
    pid.integral = limMaxInt;
  } else if (pid.integral < limMinInt) {
    pid.integral = limMinInt;
  }

  float derivative = (error - pid.previousError) / deltaTime;
  float u = (pid.kp * error + pid.ki * pid.integral + pid.kd * derivative);
  pid.previousError = error;

  //Serial.print((float)pid.kp * error *1.0);
  //Serial.print(" ");
  //Serial.print(pid.integral);
  //Serial.print(" ");
  //Serial.print(derivative);
  //Serial.print(" ");
  //Serial.print(u);
  //Serial.println(" ");
  long pwm = (static_cast<long>(u) * 4095) / MAX_VRPM;

  if (pwm > 4095) {
    pwm = 4095;
  } else if (pwm < -4095) {
    pwm = -4095;
  }
  return pwm;
}
///////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Motor_Init();
  InitEncoder();

// if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) {
//    Serial.println("Configuration failed.");
//  }
//   
   WiFi.begin(ssid,password);
   while(WiFi.status() != WL_CONNECTED)
   {
     delay(1000);
     Serial.println("Connecting to WiFi..");
   }
   Serial.println("Connected to the WiFi network");
   Serial.println(WiFi.localIP());
   wifiServer.begin(); 

}
void loop() {
  WiFiClient client = wifiServer.available();
    if (client)
   {     while(client.connected())
     {
       if (client.available() > 0)
       {
       String c = client.readStringUntil('\n');
       //Serial.println(c);
       int i=0;
       String words[4];
      c.toCharArray(message,sizeof(message));
            ptr = strtok(message," ");
       while(ptr && i<4)
       {
         words[i] = ptr;
         ptr = strtok(NULL," ");
         ++i;
       }
       
       delay(10);
       for(int i = 0;i<4 ;i++){
       speedPWM[i] = words[i].toInt(); 
       }
       //for(i=0;i<4;i++)
         //Serial.println(speedPWM[i]);
     }
     client.stop();
     //Serial.println("Client disconnected");
     }
  }
  
//  if(Serial.available()>0){ // input serial for keyboard
//    for (int i = 0; i < 4; i++) {
//      speedPWM[i] = Serial.parseInt();
//   }
//  }
  
  if (millis() - prevtime >= 100) {
    for (uint8_t i = 0; i < 4; i++) {
      Count[i].Pulse = GetEncoder(i + 1); 
      Count[i].Vrpm = ChangetoRPM(Count[i].Pulse, i);
    }
    int speedFL = PIDMOTOR(speedPWM[0],Count[0].Vrpm,100, PIDControllers[0]);
    int speedFR = PIDMOTOR(speedPWM[1],Count[1].Vrpm,100, PIDControllers[1]);
    int speedBL = PIDMOTOR(speedPWM[2],Count[2].Vrpm,100, PIDControllers[2]);
    int speedBR = PIDMOTOR(speedPWM[3],Count[3].Vrpm,100, PIDControllers[3]);
    
     for(int i = 0 ;i<4 ;i++){
        Serial.print(speedPWM[i]);//value for pc
        Serial.print(" ");
        Serial.print(Count[i].Vrpm);// value for encoder change to vrpm
        Serial.print(" ");
       
    }
    Serial.println("");
    rotateMotor(FRONT_LEFT_MOTOR,speedFL);
    rotateMotor(FRONT_RIGHT_MOTOR,speedFR);
    rotateMotor(BACK_LEFT_MOTOR,speedBL);
    rotateMotor(BACK_RIGHT_MOTOR,speedBR);
     prevtime = millis();
  }
}
///////////////////////////////////////////////////////
