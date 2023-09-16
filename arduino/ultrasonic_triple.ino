// trigger and echo pins
const int TRIGGER_LEFT = 2;
const int ECHO_LEFT = 3;

const int TRIGGER_RIGHT = 4;
const int ECHO_RIGHT = 5;

const int TRIGGER_BACK = 6;
const int ECHO_BACK = 7;

const int RASP_PI_OUT_LEFT = 10;
const int RASP_PI_OUT_RIGHT = 11;
const int RASP_PI_OUT_BACK = 12;

const int PROXIMITY_ALERT_DISTANCE = 10;

const int PULSE_RATE = 100;

void setup() {
  // setting triggers as output pins
  pinMode(TRIGGER_LEFT, OUTPUT);
  pinMode(TRIGGER_RIGHT, OUTPUT);
  pinMode(TRIGGER_BACK, OUTPUT);

  // setting echos as input pins
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_BACK, INPUT);

  // raspberry pi output pins
  pinMode(RASP_PI_OUT_LEFT, OUTPUT);
  pinMode(RASP_PI_OUT_RIGHT, OUTPUT);
  pinMode(RASP_PI_OUT_BACK, OUTPUT);
}

void loop() {
  // calculating distances for each sensor
  long distanceLeft = activateSensor(TRIGGER_LEFT, ECHO_LEFT);
  long distanceRight = activateSensor(TRIGGER_RIGHT, ECHO_RIGHT);
  long distanceBack = activateSensor(TRIGGER_BACK, ECHO_BACK);

  // LEFT SENSOR
  if (distanceLeft < PROXIMITY_ALERT_DISTANCE) {
    digitalWrite(RASP_PI_OUT_LEFT, HIGH);
  }
  else {
    digitalWrite(RASP_PI_OUT_LEFT, LOW);
  }

  // RIGHT SENSOR
  if (distanceRight < PROXIMITY_ALERT_DISTANCE) {
    digitalWrite(RASP_PI_OUT_RIGHT, HIGH);
  }
  else {
    digitalWrite(RASP_PI_OUT_RIGHT, LOW);
  }

  // BACK SENSOR
  if (distanceBack < PROXIMITY_ALERT_DISTANCE) {
    digitalWrite(RASP_PI_OUT_BACK, HIGH);
  }
  else {
    digitalWrite(RASP_PI_OUT_BACK, LOW);
  }

  // measuring each pulse cycle
  delay(PULSE_RATE);
}

long activateSensor(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  // sending out signal for 10 micro seconds
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigger, LOW);

  // waits for the echo to reach sensor
  // returns duration in microseconds
  long duration = pulseIn(echo, HIGH);

  long distanceInCm = distanceCalc(duration);

  return distanceInCm;
}

long distanceCalc(long time) {
  long result;

  // calculating cm
  result = ((time * 0.034) / 2);

  return result;
}
