#include <AFMotor.h>

// Pin definitions
#define lefts A0
#define rights A1

// Motor definitions
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Motor speed constants
const int NORMAL_SPEED = 255;  // Normal running speed
const int TURN_SPEED = 255;    // Speed during correction

void setup() {
  // Initialize sensor pins
  pinMode(lefts, INPUT);
  pinMode(rights, INPUT);
  
  // Initialize serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensors (0 is light, 1 is dark)
  int leftSensor = digitalRead(lefts);
  int rightSensor = digitalRead(rights);
  
  // Both sensors on light surface - stop
  if (leftSensor == 0 && rightSensor == 0) {
    stopRobot();
  }
  // Left sensor on dark line - correct left
  else if (leftSensor == 1 && rightSensor == 0) {
    turnLeft();
  }
  // Right sensor on dark line - correct right
  else if (leftSensor == 0 && rightSensor == 1) {
    turnRight();
  }
  // Both sensors on dark line - move forward
  else if (leftSensor == 1 && rightSensor == 1) {
    moveForward();
  }
}

void moveForward() {
  motor1.setSpeed(NORMAL_SPEED);
  motor2.setSpeed(NORMAL_SPEED);
  motor3.setSpeed(NORMAL_SPEED);
  motor4.setSpeed(NORMAL_SPEED);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft() {
  motor1.setSpeed(NORMAL_SPEED);
  motor2.setSpeed(NORMAL_SPEED);
  motor3.setSpeed(TURN_SPEED);
  motor4.setSpeed(TURN_SPEED);
  
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  motor1.setSpeed(TURN_SPEED);
  motor2.setSpeed(TURN_SPEED);
  motor3.setSpeed(NORMAL_SPEED);
  motor4.setSpeed(NORMAL_SPEED);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void stopRobot() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
