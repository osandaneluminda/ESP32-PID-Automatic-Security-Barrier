
#include <Wire.h>
#include <ESP32Servo.h>

// ---------------- PIN CONFIGURATION ----------------
#define TRIG_PIN    5
#define ECHO_PIN    18
#define SERVO_PIN   13
#define LED_PIN     2
#define BUZZER_PIN  4

#define SDA_PIN     21
#define SCL_PIN     22

// ---------------- AS5600 CONFIGURATION ----------------
#define AS5600_ADDR 0x36
#define RAW_ANGLE_H 0x0C

// ---------------- DISTANCE INTERVALS ----------------
// Three reference levels:
// Ref 70: distance > 30 cm
// Ref 40: distance > 20 cm and <= 30 cm
// Ref 0 : distance <= 20 cm

float farLimit = 30.0;
float midLimit = 20.0;
float DIST_HYST = 0.7;

// ---------------- SERVO REFERENCE POSITIONS ----------------
const int SERVO_REF0  = 80;
const int SERVO_REF40 = 120;
const int SERVO_REF70 = 150;

int servoPosition = SERVO_REF0;

// ---------------- AS5600 CALIBRATION RANGES ----------------
// Ref 0: servo 80 degrees, sensor approximately 120-130
// Ref 40: servo 120 degrees, sensor approximately 60-80
// Ref 70: servo 150 degrees, sensor approximately 105-112

float R0_MIN  = 120.0;
float R0_MAX  = 130.0;

float R40_MIN = 60.0;
float R40_MAX = 80.0;

float R70_MIN = 105.0;
float R70_MAX = 112.0;

// ---------------- PID SETTINGS ----------------
float Kp = 0.8;
float Ki = 0.03;
float Kd = 0.01;



float integral = 0;
float previousError = 0;

const float INTEGRAL_LIMIT = 100.0;
const int MAX_SPEED = 35;

// ---------------- CONTROL VARIABLES ----------------
int targetRef = 70;
int feedbackRef = -1;

float sensorAngle = 0;
float distanceCm = -1;
float error = 0;

float pTerm = 0;
float iTerm = 0;
float dTerm = 0;
float pidOutput = 0;

bool autoMode = true;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long CONTROL_INTERVAL = 50;
const unsigned long PRINT_INTERVAL = 200;

Servo gateServo;

// ---------------- AS5600 READING ----------------
float readAS5600Once() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  if (Wire.requestFrom(AS5600_ADDR, 2) != 2) {
    return -1;
  }

  uint16_t rawAngle = ((uint16_t)Wire.read() << 8) | Wire.read();
  rawAngle &= 0x0FFF;

  return rawAngle * 360.0 / 4096.0;
}

float readAS5600() {
  const int samples = 7;
  float values[samples];
  int count = 0;

  for (int i = 0; i < samples; i++) {
    float angle = readAS5600Once();

    if (angle >= 0) {
      values[count++] = angle;
    }

    delay(2);
  }

  if (count == 0) {
    return -1;
  }

  // Sort readings and use the median to reduce fluctuations.
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  return values[count / 2];
}

// ---------------- AS5600 REFERENCE CLASSIFICATION ----------------
int classifyReference(float angle) {
  if (angle < 0) {
    return -1;
  }

  if (angle >= R0_MIN && angle <= R0_MAX) {
    return 0;
  }

  if (angle >= R40_MIN && angle <= R40_MAX) {
    return 40;
  }

  if (angle >= R70_MIN && angle <= R70_MAX) {
    return 70;
  }

  // Do not invent a reference if the reading is outside all ranges.
  return -1;
}

// ---------------- ULTRASONIC DISTANCE ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

// ---------------- TARGET FROM DISTANCE ----------------
void updateTargetFromDistance(float distance) {
  if (distance < 0) {
    return;
  }

  // Explicit distance bands with a small hysteresis margin.
  if (distance > farLimit + DIST_HYST) {
    targetRef = 70;
  } else if (distance > midLimit + DIST_HYST) {
    targetRef = 70;
  } else if (distance < midLimit - DIST_HYST) {
    targetRef = 0;
  } else {
    // In the small hysteresis band, preserve the current reference.
    if (targetRef != 0 && targetRef != 40 && targetRef != 70) {
      targetRef = 40;
    }
  }

  // Ref 40 is used in the middle band.
  if (distance <= farLimit + DIST_HYST &&
      distance >= midLimit - DIST_HYST) {
    targetRef = 40;
  }

  // Explicitly select Ref 0 at the closest interval.
  if (distance <= midLimit - DIST_HYST) {
    targetRef = 0;
  }
}

// ---------------- TARGET SERVO POSITION ----------------
int getServoTarget(int ref) {
  if (ref == 0) {
    return SERVO_REF0;
  }

  if (ref == 40) {
    return SERVO_REF40;
  }

  return SERVO_REF70;
}

// ---------------- PID CONTROLLER ----------------
void runController() {
  unsigned long now = millis();

  if (now - lastControlTime < CONTROL_INTERVAL) {
    return;
  }

  float dt = (now - lastControlTime) / 1000.0;

  if (lastControlTime == 0 || dt <= 0) {
    lastControlTime = now;
    return;
  }

  lastControlTime = now;

  sensorAngle = readAS5600();
  feedbackRef = classifyReference(sensorAngle);

  int desiredServo = getServoTarget(targetRef);

  // PID feedback uses the classified AS5600 reference when available.
  // If the reading is outside the calibrated ranges, do not let that
  // classification prevent the servo from moving toward its target.
  if (feedbackRef >= 0) {
    error = (float)targetRef - (float)feedbackRef;
  } else {
    error = (float)(desiredServo - servoPosition);
  }

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - previousError) / dt;

  pTerm = Kp * error;
  iTerm = Ki * integral;
  dTerm = Kd * derivative;

  pidOutput = pTerm + iTerm + dTerm;
  previousError = error;

  // IMPORTANT FIX:
  // Movement is based on commanded servo position, not on feedbackRef.
  // This prevents an incorrect AS5600 classification from blocking Ref 0.
  int difference = desiredServo - servoPosition;

  if (difference != 0) {
    int stepSize = constrain(abs((int)pidOutput), 1, MAX_SPEED);

    // Ensure the servo continues to approach the requested position.
    if (abs(difference) < stepSize) {
      servoPosition = desiredServo;
    } else if (difference > 0) {
      servoPosition += stepSize;
    } else {
      servoPosition -= stepSize;
    }

    servoPosition = constrain(servoPosition, SERVO_REF0, SERVO_REF70);
    gateServo.write(servoPosition);
  }

  // Red LED and buzzer increase as the measured distance decreases.
  if (distanceCm >= 0 && distanceCm <= farLimit) {
    int intensity = map(
      constrain((int)(distanceCm * 10), 0, (int)(farLimit * 10)),
      (int)(farLimit * 10), 0, 0, 255
    );

    analogWrite(LED_PIN, intensity);

    if (distanceCm <= farLimit) {
      int buzzerFrequency = map(
        constrain((int)distanceCm, 0, (int)farLimit),
        (int)farLimit, 0, 500, 2200
      );

      tone(BUZZER_PIN, buzzerFrequency);
    }
  } else {
    analogWrite(LED_PIN, 0);
    noTone(BUZZER_PIN);
  }
}

// ---------------- SERIAL COMMANDS ----------------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("PID,kp,ki,kd");
  Serial.println("SET,far,mid");
  Serial.println("MODE,AUTO");
  Serial.println("MODE,MANUAL");
  Serial.println("REF,0");
  Serial.println("REF,40");
  Serial.println("REF,70");
  Serial.println("HELP");
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.equalsIgnoreCase("HELP")) {
    printHelp();
  } else if (command.equalsIgnoreCase("MODE,AUTO")) {
    autoMode = true;
    Serial.println("ACK,MODE,AUTO");
  } else if (command.equalsIgnoreCase("MODE,MANUAL")) {
    autoMode = false;
    Serial.println("ACK,MODE,MANUAL");
  } else if (command.equalsIgnoreCase("REF,0")) {
    targetRef = 0;
    Serial.println("ACK,REF,0");
  } else if (command.equalsIgnoreCase("REF,40")) {
    targetRef = 40;
    Serial.println("ACK,REF,40");
  } else if (command.equalsIgnoreCase("REF,70")) {
    targetRef = 70;
    Serial.println("ACK,REF,70");
  } else if (command.startsWith("PID,")) {
    float kp, ki, kd;

    if (sscanf(command.c_str(), "PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
      Kp = kp;
      Ki = ki;
      Kd = kd;
      integral = 0;
      previousError = 0;

      Serial.printf("ACK,PID,%.4f,%.4f,%.4f\n", Kp, Ki, Kd);
    } else {
      Serial.println("ERR,PID_FORMAT");
    }
  } else if (command.startsWith("SET,")) {
    float farValue, midValue;

    if (sscanf(command.c_str(), "SET,%f,%f", &farValue, &midValue) == 2 &&
        farValue > midValue && midValue > 0) {
      farLimit = farValue;
      midLimit = midValue;

      Serial.printf("ACK,SET,%.1f,%.1f\n", farLimit, midLimit);
    } else {
      Serial.println("ERR,SET_FORMAT");
    }
  } else {
    Serial.println("ERR,UNKNOWN_COMMAND");
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_REF0);

  servoPosition = SERVO_REF0;

  delay(500);

  Serial.println("PID Barrier Controller Ready");
  Serial.println("Distance bands: >30cm Ref70, 20-30cm Ref40, <=20cm Ref0");
  printHelp();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  handleSerial();

  float newDistance = readDistanceCm();

  if (newDistance >= 2.0 && newDistance <= 400.0) {
    distanceCm = newDistance;
  }

  if (autoMode) {
    updateTargetFromDistance(distanceCm);
  }

  runController();

  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();

    Serial.printf(
      "DATA,%lu,%.2f,%d,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s\n",
      millis(),
      distanceCm,
      targetRef,
      sensorAngle,
      feedbackRef,
      error,
      pTerm,
      iTerm,
      dTerm,
      pidOutput,
      servoPosition,
      autoMode ? "AUTO" : "MANUAL"
    );
  }
}
