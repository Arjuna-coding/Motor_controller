// ============================================================
// Welding wire feeder - CNC Shield V4 + A4988 + Arduino (Nano)
// ============================================================

// --- PIN CONFIGURATION ---
const int EN_PIN      = 8;    // A4988 enable (active LOW)
const int STEP_PIN    = 2;    // X-axis STEP
const int DIR_PIN     = 5;    // X-axis DIR
const int SWITCH_PIN  = 12;   // Feed switch to GND (SpnEn header)
const int POT_PIN     = A7;   // Speed potentiometer
const int VOLTAGE_PIN = A0;   // Arc voltage divider (10k/1k + 5V Zener)

// --- CALIBRATION ---
const float STEPS_PER_MM   = 8.72;
const float MAX_WELD_VOLTS = 55.0;   // 5V at ADC = 55V at arc

// --- SPEED SETTINGS (mm/min) ---
const float POT_MIN_SPEED  = 3550.0;
const float POT_MAX_SPEED  = 8000.0;
const float CREEP_SPEED    = 2000.0;  // Arc start / open circuit / short
const float MIN_START_SPEED = 1200.0; // Jump-start speed (avoids low-speed resonance)
const float ACCEL_RATE     = 400.0;   // mm/min added per 10 ms tick (raise if motor hums)

// --- ARC VOLTAGE THRESHOLDS (with hysteresis) ---
const float V_OPEN       = 35.0;   // Above this = no arc
const float V_SHORT      = 5.0;    // Below this = short circuit
const float V_LOW_ENTER  = 18.0;
const float V_LOW_EXIT   = 19.0;
const float V_HIGH_ENTER = 22.0;
const float V_HIGH_EXIT  = 21.0;

const float SPEED_FACTOR_LOW  = 0.7;
const float SPEED_FACTOR_HIGH = 1.3;

// --- TIMING ---
const unsigned long SENSING_INTERVAL_MS = 10;
const unsigned long DEBUG_INTERVAL_MS   = 200;
const float FILTER_ALPHA = 0.1;    // Arc voltage smoothing (0-1, lower = smoother)

// --- STATE ---
enum Zone { ZONE_NORMAL, ZONE_LOW, ZONE_HIGH };
Zone zone = ZONE_NORMAL;

unsigned long lastStepTimeMicros = 0;
unsigned long lastSensTimeMillis = 0;
unsigned long lastDebugMillis    = 0;
unsigned long stepIntervalMicros = 1000;

bool stepState = LOW;
bool feederEnabled = false;
bool wasEnabled = false;

float filteredRaw = 0.0;
float actualArcVoltage = 0.0;
float baseSpeed = POT_MIN_SPEED;
float currentRunningSpeed = 0.0;

void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);    // Driver OFF until switch is on
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);   // Forward (swap to LOW if feeding backwards)

  filteredRaw = analogRead(VOLTAGE_PIN);

  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  // ==========================================================
  // 1. SENSING, VOLTAGE LOGIC AND RAMPING (every 10 ms)
  // ==========================================================
  if (currentMillis - lastSensTimeMillis >= SENSING_INTERVAL_MS) {
    lastSensTimeMillis = currentMillis;

    // --- Switch ---
    feederEnabled = (digitalRead(SWITCH_PIN) == LOW);

    if (feederEnabled) {
      digitalWrite(EN_PIN, LOW);
      if (!wasEnabled) {
        // Jump-start above the resonance region
        currentRunningSpeed = MIN_START_SPEED;
        zone = ZONE_NORMAL;
      }
    } else {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      stepState = LOW;
      currentRunningSpeed = 0.0;
    }
    wasEnabled = feederEnabled;

    // --- Arc voltage (filtered) ---
    int rawAnalogVolts = analogRead(VOLTAGE_PIN);
    filteredRaw += FILTER_ALPHA * (rawAnalogVolts - filteredRaw);
    actualArcVoltage = (filteredRaw / 1023.0) * MAX_WELD_VOLTS;

    // --- Pot sets base speed ---
    int potValue = analogRead(POT_PIN);
    baseSpeed = POT_MIN_SPEED + (POT_MAX_SPEED - POT_MIN_SPEED) * (potValue / 1023.0);

    // --- Hysteresis zone logic ---
    switch (zone) {
      case ZONE_NORMAL:
        if (actualArcVoltage < V_LOW_ENTER)       zone = ZONE_LOW;
        else if (actualArcVoltage > V_HIGH_ENTER) zone = ZONE_HIGH;
        break;
      case ZONE_LOW:
        if (actualArcVoltage > V_LOW_EXIT) zone = ZONE_NORMAL;
        break;
      case ZONE_HIGH:
        if (actualArcVoltage < V_HIGH_EXIT) zone = ZONE_NORMAL;
        break;
    }

    // --- Target speed ---
    float targetSpeed = baseSpeed;
    if (actualArcVoltage > V_OPEN || actualArcVoltage < V_SHORT) {
      targetSpeed = CREEP_SPEED;                       // no arc or short
    } else if (zone == ZONE_HIGH) {
      targetSpeed = baseSpeed * SPEED_FACTOR_HIGH;
    } else if (zone == ZONE_LOW) {
      targetSpeed = baseSpeed * SPEED_FACTOR_LOW;
    }

    // --- Soft-start ramp ---
    if (feederEnabled) {
      if (currentRunningSpeed < targetSpeed) {
        currentRunningSpeed += ACCEL_RATE;
        if (currentRunningSpeed > targetSpeed) currentRunningSpeed = targetSpeed;
      } else if (currentRunningSpeed > targetSpeed) {
        currentRunningSpeed -= ACCEL_RATE;
        if (currentRunningSpeed < targetSpeed) currentRunningSpeed = targetSpeed;
      }

      // --- Convert speed to step timing ---
      float stepsPerSecond = (currentRunningSpeed / 60.0) * STEPS_PER_MM;
      if (stepsPerSecond > 1.0) {
        stepIntervalMicros = (unsigned long)(1000000.0 / stepsPerSecond / 2.0);
      }
    }
  }

  // ==========================================================
  // 2. NON-BLOCKING STEP PULSE GENERATION
  // ==========================================================
  if (feederEnabled && currentRunningSpeed > 0.0) {
    unsigned long currentMicros = micros();
    if (currentMicros - lastStepTimeMicros >= stepIntervalMicros) {
      lastStepTimeMicros = currentMicros;
      stepState = !stepState;
      digitalWrite(STEP_PIN, stepState);
    }
  }

  // ==========================================================
  // 3. DEBUG OUTPUT (for tuning)
  // ==========================================================
  if (currentMillis - lastDebugMillis >= DEBUG_INTERVAL_MS) {
    lastDebugMillis = currentMillis;
    Serial.print("V=");     Serial.print(actualArcVoltage, 1);
    Serial.print(" base="); Serial.print(baseSpeed, 0);
    Serial.print(" run=");  Serial.print(currentRunningSpeed, 0);
    Serial.print(" zone="); Serial.println((int)zone);
  }
}
