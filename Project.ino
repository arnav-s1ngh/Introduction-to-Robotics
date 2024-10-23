// Demo-1 :- Straight Line Path can be traversed easily without PID
#include <AFMotor.h>
// Reference :- https://quadstore.in/wp-content/uploads/2020/07/LINE_FOLLOWING_CAR.zip
// Pin definitions
#define lefts A1
#define center A2
#define rights A0

// Motors
//Right
AF_DCMotor motor1(1,MOTOR12_1KHZ); 
AF_DCMotor motor2(2,MOTOR12_1KHZ);
//Left
AF_DCMotor motor3(3,MOTOR34_1KHZ);
AF_DCMotor motor4(4,MOTOR34_1KHZ);

// Motor speed 
const int speed=255;

void setup() {
  pinMode(lefts,INPUT);
  pinMode(center,INPUT);
  pinMode(rights,INPUT);
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
  while(digitalRead(center) == 0) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  haltbot();
}

void loop() {
  int leftVal=digitalRead(lefts);
  int centerVal=digitalRead(center);
  int rightVal=digitalRead(rights);
  if (centerVal == 1) {
    moveforward();
  }
  else if (leftVal == 1) {
    turnleft();
  }
  else if (rightVal == 1) {
    turnright();
  }
  else {
    haltbot();
  }
}

void moveforward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnleft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnright() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void haltbot() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

