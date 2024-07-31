#include <PID_v1.h> // PID by Brett Beauregard
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_SPEED 70

// MPU
MPU6050 mpu;

// control / status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation / motion vars
Quaternion q;         // [w, x, y, z] quaternion container
VectorFloat gravity;  // [x, y, z] gravity vector
float ypr[3];         // [yaw, pitch, roll] yaw / pitch / roll container and gravity vector

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

// interrupt service routine
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID
double originalSetpoint = 178.0; // 178 for stillstand, modify to make robot drive in a direction
double setpoint = originalSetpoint;
double movingAngleOffset = 0.0;
double input, output;

double Kp = 56; // 56
double Kd = 2.1; // 2.4
double Ki = 450; // 500
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// MOTORS
// motor LEFT
int enA = 9; // PWM pin
int in1 = 4;
int in2 = 5;

// motor RIGHT
int enB = 10; // PWM pin
int in3 = 6;
int in4 = 7;

// LED
int ledPin = 13; // built-in LED 

double currentSpeed = 0;

void runMotors(double speed) {
  // if speed doesn't change don't do anything
  if (speed == currentSpeed) {
    return;
  }

  // move backwards
  if (speed < 0) {
    if (speed > -MIN_SPEED) {
      speed = -MIN_SPEED;
    }

    // LEFT motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // RIGHT motor
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    // speed ranges from 0 - 255 -> min 70
    analogWrite(enA, abs(speed)); 
    analogWrite(enB, abs(speed));
  }
  // move forwards
  else {
    if (speed < MIN_SPEED) {
      speed = MIN_SPEED;
    }

    // LEFT motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // RIGHT motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    // speed ranges from 0 - 255 -> min 70
    analogWrite(enA, speed); 
    analogWrite(enB, speed);
  }

  currentSpeed = speed;
}


void setup() {
  Serial.begin(9600);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // own gyro offsets previously meassured
  mpu.setXGyroOffset(93);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(-43);
  mpu.setZAccelOffset(943);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // MOTOR SETUP
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // LED SETUP
  pinMode(ledPin, OUTPUT); // initialize LED pin as output

  // Turn on LED
  digitalWrite(ledPin, HIGH);
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // no mpu data
    // performing PID calculations
    pid.Compute();

    // sending output to motors
    runMotors(output);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // change from ypr[1] to ypr[2] to use the roll angle
    input = ypr[2] * 180 / M_PI + 180;
  }
}
