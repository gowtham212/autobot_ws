/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Tick Data Publishing Variables and Constants ///////////////

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 15
#define ENC_IN_RIGHT_A 5
#define RXD2 16
#define TXD2 17
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 18

boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

// Motor A connections
const int enA = 13;
const int in1 = 12;
const int in2 = 14;

// Motor B connections
const int enB = 27;
const int in3 = 26;
const int in4 = 25;

const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle = 200;
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 620;

// Wheel radius in meters
const double WHEEL_RADIUS = 0.033;

// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.17;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 3100; // Originally 2880

// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
const int K_P = 400;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 70;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 167;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 130;  // about 0.1 m/s
const int PWM_MAX = 225; // about 0.172 m/s

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

/////////////////////// Tick Data Publishing Functions ////////////////////////

// Increment the number of ticks
void right_wheel_tick()
{

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW)
  {
    Direction_right = false; // Reverse
  }
  else
  {
    Direction_right = true; // Forward
  }

  if (Direction_right)
  {

    if (right_wheel_tick_count.data == encoder_maximum)
    {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else
    {
      right_wheel_tick_count.data++;
    }
  }
  else
  {
    if (right_wheel_tick_count.data == encoder_minimum)
    {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else
    {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick()
{

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW)
  {
    Direction_left = true; // Reverse
  }
  else
  {
    Direction_left = false; // Forward
  }

  if (Direction_left)
  {
    if (left_wheel_tick_count.data == encoder_maximum)
    {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else
    {
      left_wheel_tick_count.data++;
    }
  }
  else
  {
    if (left_wheel_tick_count.data == encoder_minimum)
    {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else
    {
      left_wheel_tick_count.data--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////
// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
// Calculate the left wheel linear velocity in m/s every time a
// tick count message is rpublished on the /left_ticks topic.
void calc_vel_left_wheel()
{
  //  Serial2.println("recieved value");
  // Previous timestamp
  static double prevTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000)
  {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);

  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000);
}

// Calculate the right wheel linear velocity in m/s every time a
// tick count message is published on the /right_ticks topic.
void calc_vel_right_wheel()
{

  // Previous timestamp
  static double prevTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 10000)
  {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);
  // Serial2.println(velRightWheel);
  prevRightCount = right_wheel_tick_count.data;

  prevTime = (millis() / 1000);
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist &cmdVel)
{
  // Serial2.println("recieved values");
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000);
 // Cap values at [-1 .. 1]
  float x = max(min(cmdVel.linear.x, 1.0f), -1.0f);
  float z = max(min(cmdVel.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (x - z) / 2;
  float r = (x + z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWM_MAX);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWM_MAX);

  // Set direction pins and PWM
  digitalWrite(in1, l > 0);
  digitalWrite(in2, l < 0);
  digitalWrite(in3, r > 0);
  digitalWrite(in4, r < 0);
  ledcWrite(pwmChannel, rPwm);
  ledcWrite(pwmChannel1, lPwm);
  Serial2.println("leftPwm:"+String(lPwm)+"right pwm:"+String(rPwm));


}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup()
{

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set the motor speed
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enA, pwmChannel);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enB, pwmChannel1);
  ledcWrite(pwmChannel, 0);

  ledcWrite(pwmChannel1, 0);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  Serial2.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.print("hello");
}

void loop()
{

  nh.spinOnce();

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval)
  {

    previousMillis = currentMillis;

    // Publish tick counts to topics
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);

    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }

  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

}