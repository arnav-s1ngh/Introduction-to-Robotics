#include <AFMotor.h>

// Pin definitions
#define lefts A1
#define center A2
#define rights A0

// Motors
//Right
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
//Left
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Motor speed 
const int speed = 255;

void setup() {
  // Set up sensor pins as inputs
  pinMode(lefts, INPUT);
  pinMode(center, INPUT);
  pinMode(rights, INPUT);
  
  // Set initial motor speeds
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
  
  // Start by finding the line if not on it
  while(digitalRead(center) == 0) {  // While center sensor sees light
    // Move forward until line is found
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  
  // Stop once line is found
  stopMotors();
}

void loop() {
  // Read sensor values
  int leftVal = digitalRead(lefts);
  int centerVal = digitalRead(center);
  int rightVal = digitalRead(rights);
  
  // Line following logic
  if (centerVal == 1) {
    // On the line, move forward
    moveForward();
  }
  else if (leftVal == 1) {
    // Line is to the left, turn left
    turnLeft();
  }
  else if (rightVal == 1) {
    // Line is to the right, turn right
    turnRight();
  }
  else {
    // No line detected, stop
    stopMotors();
  }
}

// Motor control functions
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnRight() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

