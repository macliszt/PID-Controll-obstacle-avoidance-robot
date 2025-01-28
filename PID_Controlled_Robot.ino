#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h>
#include <PID_v1.h>  // Include the PID library

// Define the Trig and the Echo pins
#define TRIG_PIN A0 
#define ECHO_PIN A1 

#define MAX_DISTANCE 200 
#define MAX_SPEED 120 // sets speed of DC motors
#define MAX_SPEED_OFFSET 20

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // The NewPing library takes in the trig and the echo pins and also the maximum distance

AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo servo;   

boolean goesForward = false;
int distance = 100;
int speedSet = 0;

// PID constants
double Kp = 2.0;  // Proportional gain 2
double Ki = 5.0;  // Integral gain 5
double Kd = 1.0;  // Derivative gain 1


int integralTerm = 0;  // Integral term to store accumulated error
int prevError = 0;     // Previous error to calculate derivative
int maxIntegral = 100; // Maximum allowed value for the integral term (to avoid windup)

// PID variables
double setpoint = 30;  // Desired distance from the obstacle (in cm)
double input = 0;      // Current distance from the obstacle
double output = 0;     // PID output for adjusting speed

// Create the PID object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);  // Use the defined Kp, Ki, Kd values

void setup() {
  servo.attach(9);  
  servo.write(115); 
  delay(2000);

  distance = readPing();  // Initial distance reading
  delay(100);

  // Initialize PID
  pid.SetMode(AUTOMATIC);  // Set PID to automatic mode
  pid.SetOutputLimits(-MAX_SPEED, MAX_SPEED);  // Set limits for PID output (speed range)
  pid.SetSampleTime(50);  // Update PID every 50ms
}

void loop() {
  int distanceR = 0;
  int distanceL =  0;
  delay(40);

  int error = distance - distance; // Calculate the error (desired target distance)
  
  // Anti-Windup: Prevent the integral term from exceeding limits
  integralTerm += error;  // Accumulate the error
  
  if (integralTerm > maxIntegral) integralTerm = maxIntegral;   // Limit the integral term
  if (integralTerm < -maxIntegral) integralTerm = -maxIntegral; // Limit the integral term

  int derivativeTerm = error - prevError;  // Calculate the derivative term
  prevError = error;  // Update the previous error for the next iteration

  // PID output calculation
  int pidOutput = Kp * error + Ki * integralTerm + Kd * derivativeTerm;

  // Apply the PID output to control the robot (this could control speed, direction, etc.)
  if (distance <= 30) {
    Stop(); // stop the car
    delay(100);
    moveBackward();
    delay(300);
    Stop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL) {
      turnRight();
      Stop();
    } else {
      turnLeft();
      Stop();
    }
  } else {
    moveForward();
  }

  distance = readPing(); // Update the distance reading
}


int lookRight() {
    servo.write(30); 
    delay(500);
    int distance = readPing();
    delay(100);
    servo.write(115); 
    return distance;
}

int lookLeft() {
    servo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    servo.write(115); 
    return distance;
}

// For the Ultrasonic sensor to measure the distance
int readPing() { 
  delay(70);
  int cm = 0;
  for (int i = 0; i < 5; i++) {  // Take 5 readings
    cm += sonar.ping_cm();
    delay(30);
  }
  cm = cm / 5;  // Average the readings
  if(cm == 0) {
    cm = 250;
  }
  return cm;
}


// To stop the car
void Stop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
} 

// For the car to move Forward with PID control
void moveForward() {
  if (!goesForward) {
    goesForward = true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    motor3.run(FORWARD); 
    motor4.run(FORWARD);     
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) { // Gradual acceleration
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

// Move backward with PID control
void moveBackward() {
    goesForward = false;
    motor1.run(BACKWARD);      
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);  
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
}  

// To turn right and move in right direction
void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(500);
  motor1.run(FORWARD);      
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);      
} 

// To turn left and move in left direction
void turnLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(500);
  motor1.run(FORWARD);     
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}  
