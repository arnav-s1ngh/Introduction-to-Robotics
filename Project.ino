//Project-1: //ARDUINO LINE FOLLOWING CAR - QUAD Robotics - A unit of Quad Store//
//www.quadstore.in

// YOU HAVE TO INSTALL THE AFMOTOR LIBRARY BEFORE UPLOAD THE CODE//
// GO TO SKETCH >> INCLUDE LIBRARY >> ADD .ZIP LIBRARY >> SELECT AF MOTOR ZIP FILE //


//including the libraries
#include <AFMotor.h>

// Defining pins and variables
#define lefts A0
#define center A1
#define rights A2

#define baseSpeed 200  // Base speed for motors

// Defining motors
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// PID variables
float Kp = 1.5;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 1.0;  // Derivative gain
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  // Setting motor speeds
  motor1.setSpeed(baseSpeed);
  motor2.setSpeed(baseSpeed);
  motor3.setSpeed(baseSpeed);
  motor4.setSpeed(baseSpeed);
  
  pinMode(lefts, INPUT);
  pinMode(center, INPUT);
  pinMode(rights, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Sensor readings
  int leftValue = analogRead(lefts);
  int centerValue = analogRead(center);
  int rightValue = analogRead(rights);

  // Error calculation (assume line detection threshold is 255)
  float error = 0;
  if (leftValue <= 255) {
    error = 1;  // Line on left
  } else if (rightValue <= 255) {
    error = -1; // Line on right
  } else if (centerValue <= 255) {
    error = 0;  // Line in the center
  }

  // Time difference
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  lastTime = currentTime;

  // PID calculations
  integral += error * deltaTime;                 // Integral term
  float derivative = (error - previousError) / deltaTime;  // Derivative term
  previousError = error;

  // Calculate PID output
  float pid = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speeds based on PID output
  int leftMotorSpeed = baseSpeed - pid;
  int rightMotorSpeed = baseSpeed + pid;

  // Constrain motor speeds within the allowable range (0-255)
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Set motor speeds
  motor1.setSpeed(leftMotorSpeed);
  motor2.setSpeed(leftMotorSpeed);
  motor3.setSpeed(rightMotorSpeed);
  motor4.setSpeed(rightMotorSpeed);

  // Run motors
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Debugging output
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(pid);
  Serial.print(" | Left Motor: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right Motor: ");
  Serial.println(rightMotorSpeed);

  delay(50);  // Loop delay
}
