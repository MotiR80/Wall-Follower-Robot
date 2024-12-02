#include <PID_v1.h>
#include <AFMotor.h>

// Distance Sensor
#define trigPin_F A0    // Define the digital pin for the trigger pin of the SRF04 sensor
#define echoPin_F A1   // Define the digital pin for the echo pin of the SRF04 sensor
#define trigPin_R A2    // Define the digital pin for the trigger pin of the SRF04 sensor
#define echoPin_R A3   // Define the digital pin for the echo pin of the SRF04 sensor

// MotorDC Initialize
#define BASE_SPEED 128
const int MOTOR_1 = 1; 
const int MOTOR_2 = 2; 
const int MOTOR_3 = 3; 
const int MOTOR_4 = 4; 

AF_DCMotor motor1(MOTOR_1, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor2(MOTOR_2, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor3(MOTOR_3, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor4(MOTOR_4, MOTOR12_64KHZ); // create motor object, 64KHz pwm
//===============================================================================
//  Initialization
//===============================================================================

// PID parameters
double Setpoint, Input, Output;
double last_right_distance;

// PID instance
double Kp=2.5, Ki=0.0, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Desired distance from the wall (in cm)
int maxForwardDist = 25;

void setup() {

  // Ultrasonic Sensor Pins
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_F, INPUT);
  pinMode(trigPin_R, OUTPUT);
  pinMode(echoPin_R, INPUT);

  Input = getDistance(trigPin_R, echoPin_R);
  Setpoint = 20;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127, 127);

  // Motors Base Speed
  setSpeed(BASE_SPEED, BASE_SPEED);
  delay(1000);
}

void loop() {
  controlDistance();
}


void controlDistance(){
  double distance_forward = getDistance(trigPin_F, echoPin_F);

   if (distance_forward > maxForwardDist){
    Input = getDistance(trigPin_R, echoPin_R);

    // Compute PID output
    myPID.Compute();

// Adjust motor speeds based on PID output
    int rightWeelsSpeed = constrain(BASE_SPEED + Output, 0, 255);
    int leftWeelsSpeed = constrain(BASE_SPEED - Output, 0, 255);
    // Set motor speeds
    setSpeed(leftWeelsSpeed, rightWeelsSpeed);
    
    moveForward();
  }
  else{
    turnLeft();
  }
  return Output;
}

void setSpeed(int leftWeelsSpeed, int rightWeelsSpeed){
  motor1.setSpeed(leftWeelsSpeed);          // set the motor speed to 0-255
  motor2.setSpeed(leftWeelsSpeed);
  motor3.setSpeed(rightWeelsSpeed);
  motor4.setSpeed(rightWeelsSpeed);
}

void moveForward() {
  motor1.run(FORWARD);  // move forward
  motor2.run(FORWARD);
  motor3.run(FORWARD); 
  motor4.run(FORWARD);
}

void turnLeft(){
  setSpeed(0, 255);
  motor1.run(RELEASE);   // turn left
  motor2.run(RELEASE);
  motor3.run(FORWARD); 
  motor4.run(FORWARD);
  delay(250);
}

double getDistance(int trigpin1, int echopin1) {
  // Send a 10us pulse to trigger pin
  digitalWrite(trigpin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin1, LOW);
  
  // Read the echo pin and calculate the distance
  long duration = pulseIn(echopin1, HIGH);
  double distance = duration / 58;
  return distance;
}