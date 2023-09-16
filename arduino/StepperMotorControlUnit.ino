#include <AccelStepper.h>

/* STEPPER MOTOR */
/* 800 steps = ONE FULL ROTATION -> steppers are set to quarter step resolution */

// setting up steppers with corresponding hardware pins
AccelStepper StepperX(1, 2, 5);
AccelStepper StepperY(1, 3, 6);
AccelStepper StepperZ(1, 4, 7);

// States: movingForwards, movingBackwards, turningLeft, turningRight, stopped
String moveState = "stopped";
String previousState = "stopped";

const int SPEED = 250;

void setup() {
  StepperX.setMaxSpeed(1000);
  StepperY.setMaxSpeed(1000);
  StepperZ.setMaxSpeed(1000);

  // initialize serial communication
  Serial.begin(115200);

  // wait for serial communication to initiate 
  while (!Serial) {

  }
}

void loop() {
  // read data if available
  if (Serial.available() > 0) {
    String inputMessage = Serial.readStringUntil('\n');

    previousState = moveState;

    moveState = inputMessage;
  }

  // sending state to raspberry pi
  if (moveState == "REQUEST_STATE") {
    Serial.println(previousState);
    moveState = previousState;
  }

  if (moveState == "movingForwards") {
    StepperX.setSpeed(SPEED);
    StepperY.setSpeed(SPEED);
    StepperZ.setSpeed(0);

    StepperX.runSpeed();
    StepperY.runSpeed();
    StepperZ.runSpeed();
  }
  else if (moveState == "movingBackwards") {
    StepperX.setSpeed(-SPEED);
    StepperY.setSpeed(-SPEED);
    StepperZ.setSpeed(0);

    StepperX.runSpeed();
    StepperY.runSpeed();
    StepperZ.runSpeed();
  }
  else if (moveState == "turningLeft") {
    StepperX.setSpeed(-SPEED);
    StepperY.setSpeed(SPEED);
    StepperZ.setSpeed(SPEED);

    StepperX.runSpeed();
    StepperY.runSpeed();
    StepperZ.runSpeed();
  }
  else if (moveState == "turningRight") {
    StepperX.setSpeed(SPEED);
    StepperY.setSpeed(-SPEED);
    StepperZ.setSpeed(-SPEED);

    StepperX.runSpeed();
    StepperY.runSpeed();
    StepperZ.runSpeed();
  }
  else if (moveState == "stopped") {
    StepperX.stop();
    StepperY.stop();
    StepperZ.stop();
  }
}
