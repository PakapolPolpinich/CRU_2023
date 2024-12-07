#include <ESP32Encoder.h>

ESP32Encoder FRONTEncoderLEFT;
ESP32Encoder FRONTEncoderRIGHT;
ESP32Encoder BACKEncoderLEFT;
ESP32Encoder BACKEncoderRIGHT;

#define EN1_LEFT_F 36
#define EN2_LEFT_F 39
#define EN1_RIGHT_F 22
#define EN2_RIGHT_F 23
#define EN1_LEFT_B 34
#define EN2_LEFT_B 35
#define EN1_RIGHT_B 19
#define EN2_RIGHT_B 21

void InitEncoder() {
  FRONTEncoderLEFT.attachFullQuad(EN1_LEFT_F, EN2_LEFT_F);
  FRONTEncoderRIGHT.attachFullQuad(EN1_RIGHT_F, EN2_RIGHT_F);
  BACKEncoderLEFT.attachFullQuad(EN1_LEFT_B, EN2_LEFT_B);
  BACKEncoderRIGHT.attachFullQuad(EN1_RIGHT_B, EN2_RIGHT_B);

 
  FRONTEncoderLEFT.setCount(0);
  FRONTEncoderRIGHT.setCount(0);
  BACKEncoderLEFT.setCount(0);
  BACKEncoderRIGHT.setCount(0);
}

long GetEncoder(uint8_t i){
  switch (i){
    case 1:
      return FRONTEncoderLEFT.getCount();
    case 2:
      return FRONTEncoderRIGHT.getCount();
    case 3:
      return BACKEncoderLEFT.getCount();
    case 4:
      return BACKEncoderRIGHT.getCount();      
  }  
}
