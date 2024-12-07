 #include <ESP32Servo.h>

Servo rail;
Servo arm;
Servo lock;

int currentposition = 1;

void Servoinit(){
  rail.attach(13);
  arm.attach(27);
  lock.attach(14);
  arm.write(110);// wait for object
}


void RailServo(int Target) {
  if (Target != currentposition) {
    int duration = abs(Target - currentposition) * 100 ;
      if (Target > currentposition) {
        rail.write(0);
      }
      else if (Target < currentposition) {
        rail.write(3000);
      }
      delay(duration);
      rail.write(1500);
      currentposition = Target;
  }
}//1-12

void UpDownServo(int armPosition) {
  if (armPosition > 175) {
    arm.write(175);  // Set to the maximum position (orthogonal)
  } else if (armPosition < 110) {
    arm.write(110);  // Set to the minimum position (wait for object)
  } else {
    arm.write(armPosition);  // Set to the specified position
  }
}

void Lock(int door){
  if (door == 0){//lock
    lock.write(0);
  } else if (door == 1){
    lock.write(75);
  }
}
