#include <Arduino.h>
#include <math.h>

// SENTINEL: Adaptive irrigation controller
// ESP32 + soil moisture input + pump indicator LED
// LED indicates the pump command; it is not a physical pump.
//
// TIME_SCALE = 60:
// 1 simulated minute = approximately 1 real second.

const int SENSOR_PIN = 34;
const int SENSOR_POWER_PIN = 25;
const int PUMP_LED_PIN = 26;

const float TIME_SCALE = 60.0f;
const float DRY_V = 2.80f;
const float WET_V = 1.20f;

const float START_M = 35.0f;
const float RELEASE_M = 45.0f;

const unsigned long SETTLE_MS = 100;
const unsigned long CONFIRM_MS =
    max(1UL, (unsigned long)(30000.0f / TIME_SCALE));
const unsigned long MIN_RUN_MS =
    max(1UL, (unsigned long)(300000.0f / TIME_SCALE));
const unsigned long DAY_MS =
    max(1UL, (unsigned long)(86400000.0f / TIME_SCALE));

enum Mode { ADAPTIVE, FIXED };

enum State {
  OFF_READY,
  DRY_CONFIRM,
  ON_MINIMUM,
  ON_TRACKING,
  START_LOCKOUT,
  SENSOR_FAULT
};

Mode mode = ADAPTIVE;
State state = OFF_READY;

bool pump = false;
bool forcedFault = false;
bool testInput = false;
bool pending = false;
bool validLast = false;
bool qualityBad = false;

float testMoisture = 50;
float lastV = NAN;
float lastM = NAN;
float prevM = NAN;
float spreadV = 0;

unsigned long lastRead = 0;
unsigned long nextReadDue = 0;
unsigned long pumpStart = 0;
unsigned long confirmDue = 0;
unsigned long dryStart = 0;
unsigned long dayStart = 0;
unsigned long reads = 0;

unsigned int startsToday = 0;
unsigned int scheduledInterval = 10;

String cmd;

// --------------------------------------------------
// TIME AND STATE HELPERS
// --------------------------------------------------

unsigned long minutesMs(unsigned int m) {
  return (unsigned long)(m * 60000.0f / TIME_SCALE);
}

const char* stateText(State s) {
  switch (s) {
    case OFF_READY: return "OFF_READY";
    case DRY_CONFIRM: return "DRY_CONFIRM";
    case ON_MINIMUM: return "ON_MINIMUM";
    case ON_TRACKING: return "ON_TRACKING";
    case START_LOCKOUT: return "START_LOCKOUT";
    case SENSOR_FAULT: return "SENSOR_FAULT";
  }
  return "UNKNOWN";
}

float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

float toMoisture(float v) {
  return clampf(
      100.0f * (DRY_V - v) / (DRY_V - WET_V),
      0,
      100
  );
}

// --------------------------------------------------
// ADAPTIVE SAMPLING INTERVAL
// --------------------------------------------------

unsigned int nextInterval() {

  // Path A: fixed five-minute reference
  if (mode == FIXED) {
    return 5;
  }

  // Pump running: sample every simulated minute
  if (pump) {
    return 1;
  }

  // No valid reading yet: use a cautious short interval
  if (!validLast) {
    return 1;
  }

  // Measurement quality is uncertain
  if (qualityBad) {
    return 1;
  }

  // Near the dry-start or release threshold
  if (fabsf(lastM - START_M) <= 5.0f ||
      fabsf(lastM - RELEASE_M) <= 5.0f) {
    return 1;
  }

  // Moisture is trending downward
  if (!isnan(prevM) && lastM < prevM - 0.5f) {
    return 5;
  }

  // Wet, stable measurements
  if (lastM >= 55.0f) {
    return 10;
  }

  // Intermediate moisture: use five minutes
  return 5;
}

const char* intervalReason() {

  if (mode == FIXED) {
    return "FIXED_REFERENCE";
  }

  if (pump) {
    return "PUMP_OPERATING";
  }

  if (!validLast) {
    return "NO_VALID_MEASUREMENT_YET";
  }

  if (qualityBad) {
    return "UNCERTAIN_MEASUREMENT";
  }

  if (fabsf(lastM - START_M) <= 5.0f ||
      fabsf(lastM - RELEASE_M) <= 5.0f) {
    return "NEAR_THRESHOLD";
  }

  if (!isnan(prevM) && lastM < prevM - 0.5f) {
    return "TRENDING_TOWARD_DRYNESS";
  }

  if (lastM >= 55.0f) {
    return "WET_STABLE";
  }

  return "INTERMEDIATE_MOISTURE";
}

// Schedule and print the actual selected interval
void scheduleNextRead() {

  scheduledInterval = nextInterval();

  nextReadDue = millis() + minutesMs(scheduledInterval);

  Serial.printf(
      "[ADAPTIVE] Condition=%s; "
      "next_sampling_interval=%u simulated minute(s); "
      "real_delay=%.2f second(s)\n",
      intervalReason(),
      scheduledInterval,
      60.0f * scheduledInterval / TIME_SCALE
  );
}

// --------------------------------------------------
// PUMP COMMAND
// --------------------------------------------------

void setPump(bool on, const char* why) {

  if (pump == on) {
    return;
  }

  pump = on;

  digitalWrite(
      PUMP_LED_PIN,
      on ? HIGH : LOW
  );

  if (on) {

    pumpStart = millis();
    startsToday++;

    Serial.printf(
        "[ACTUATOR] PUMP_ON reason=%s starts_today=%u\n",
        why,
        startsToday
    );

  } else {

    Serial.printf(
        "[ACTUATOR] PUMP_OFF reason=%s "
        "runtime_sim_seconds=%.1f\n",
        why,
        (millis() - pumpStart) * TIME_SCALE / 1000.0f
    );
  }
}

// --------------------------------------------------
// DAILY START COUNTER
// --------------------------------------------------

void updateDay() {

  if (millis() - dayStart >= DAY_MS) {

    dayStart = millis();
    startsToday = 0;

    Serial.println(
        "[DAY] Simulated accounting day reset."
    );

    if (state == START_LOCKOUT) {
      state = OFF_READY;
    }
  }
}

// --------------------------------------------------
// SENSOR ACQUISITION
// --------------------------------------------------

bool readSensor(float &v, float &m, float &sp) {

  if (forcedFault) {
    return false;
  }

  // Serial test input for Wokwi scenarios
  if (testInput) {

    m = clampf(testMoisture, 0, 100);

    v = DRY_V -
        (m / 100.0f) * (DRY_V - WET_V);

    sp = 0;

    return true;
  }

  // Power the sensor before reading
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(SETTLE_MS);

  const int N = 8;

  int a[N];
  int lo = 4095;
  int hi = 0;

  for (int i = 0; i < N; i++) {

    a[i] = analogRead(SENSOR_PIN);

    if (a[i] < lo) lo = a[i];
    if (a[i] > hi) hi = a[i];

    delay(2);
  }

  // Turn sensor power off after acquisition
  digitalWrite(SENSOR_POWER_PIN, LOW);

  // Sort readings
  for (int i = 0; i < N - 1; i++) {

    for (int j = i + 1; j < N; j++) {

      if (a[j] < a[i]) {

        int t = a[i];
        a[i] = a[j];
        a[j] = t;
      }
    }
  }

  // Median of the middle two readings
  int med = (a[3] + a[4]) / 2;

  v = med / 4095.0f * 3.3f;

  sp = (hi - lo) / 4095.0f * 3.3f;

  m = toMoisture(v);

  return v >= 0.05f && v <= 3.25f;
}

// --------------------------------------------------
// MAIN ACQUISITION AND CONTROL LOGIC
// --------------------------------------------------

void acquire() {

  float v = NAN;
  float m = NAN;
  float sp = 0;

  lastRead = millis();

  bool ok = readSensor(v, m, sp);

  reads++;

  // Invalid sensor reading
  if (!ok) {

    Serial.printf(
        "[READING] n=%lu valid=NO state=%s\n",
        reads,
        stateText(state)
    );

    state = SENSOR_FAULT;
    pending = false;
    validLast = false;

    setPump(false, "sensor_fault");

    Serial.println(
        "[FAULT] Invalid sensor; pump commanded OFF."
    );

    scheduleNextRead();

    return;
  }

  float old = lastM;

  lastV = v;
  lastM = m;
  spreadV = sp;

  qualityBad = sp > 0.20f;
  validLast = true;

  Serial.printf(
      "[READING] n=%lu valid=YES voltage_V=%.3f "
      "moisture_pct=%.1f spread_V=%.3f state=%s\n",
      reads,
      v,
      m,
      sp,
      stateText(state)
  );

  if (qualityBad) {

    Serial.println(
        "[QUALITY] UNCERTAIN burst spread; "
        "short interval selected."
    );
  }

  // Recover automatically when the reading becomes valid
  if (state == SENSOR_FAULT) {

    state = OFF_READY;

    Serial.println(
        "[RECOVERY] Sensor input valid again."
    );
  }

  // ----------------------------------------------
  // DRY-SOIL CONFIRMATION
  // ----------------------------------------------

  if (!pump &&
      m <= START_M &&
      state != START_LOCKOUT) {

    // First dry reading: schedule confirmation
    if (!pending) {

      pending = true;

      dryStart = millis();

      confirmDue = millis() + CONFIRM_MS;

      state = DRY_CONFIRM;

      Serial.println(
          "[CONFIRM] Dry threshold detected; "
          "fresh confirmation scheduled."
      );

      // The confirmation timer takes precedence
      scheduleNextRead();

      return;
    }

    // Second time-separated dry acquisition confirms
    pending = false;

    Serial.printf(
        "[LATENCY] confirmation_elapsed_sim_seconds=%.1f\n",
        (millis() - dryStart) * TIME_SCALE / 1000.0f
    );

    // Enforce daily start limit
    if (startsToday >= 10) {

      state = START_LOCKOUT;

      Serial.println(
          "[LOCKOUT] Start denied: daily limit "
          "of 10 reached; no eleventh start."
      );

      scheduleNextRead();

      return;
    }

    setPump(
        true,
        "confirmed_moisture_le_35_pct"
    );

    state = ON_MINIMUM;

    prevM = old;

    scheduleNextRead();

    return;
  }

  // If the next valid reading is above 35%,
  // cancel pending confirmation
  if (pending && m > START_M) {

    pending = false;

    state = OFF_READY;

    Serial.println(
        "[CONFIRM] Dry condition not confirmed; no start."
    );
  }

  // ----------------------------------------------
  // PUMP RUNNING CONTROL
  // ----------------------------------------------

  if (pump) {

    unsigned long elapsed = millis() - pumpStart;

    if (elapsed < MIN_RUN_MS) {

      state = ON_MINIMUM;

      Serial.printf(
          "[GUARD] Minimum runtime active; "
          "elapsed_sim_seconds=%.1f required=300\n",
          elapsed * TIME_SCALE / 1000.0f
      );

    } else {

      state = ON_TRACKING;

      if (m >= RELEASE_M) {

        setPump(
            false,
            "moisture_ge_45_after_minimum"
        );

        state = OFF_READY;
      }
    }

  } else if (startsToday >= 10 && m <= START_M) {

    state = START_LOCKOUT;

  } else if (state != DRY_CONFIRM) {

    state = OFF_READY;
  }

  prevM = old;

  // Print and schedule the next interval
  scheduleNextRead();
}

// --------------------------------------------------
// STATUS AND HELP
// --------------------------------------------------

void status() {

  Serial.printf(
      "\n[STATUS] mode=%s state=%s pump=%s "
      "starts_today=%u/10 reads=%lu\n",
      mode == ADAPTIVE ? "ADAPTIVE" : "FIXED_5_MIN",
      stateText(state),
      pump ? "ON" : "OFF",
      startsToday,
      reads
  );

  Serial.printf(
      "input=%s fault=%s scale=%.1f "
      "(1 simulated minute = %.2f real seconds)\n",
      testInput ? "SERIAL_TEST" : "POTENTIOMETER",
      forcedFault ? "ON" : "OFF",
      TIME_SCALE,
      60.0f / TIME_SCALE
  );

  if (validLast) {

    Serial.printf(
        "last_voltage=%.3f V last_moisture=%.1f%% "
        "quality_uncertain=%s\n",
        lastV,
        lastM,
        qualityBad ? "YES" : "NO"
    );
  }

  Serial.printf(
      "next_sampling_interval=%u simulated minute(s); "
      "reason=%s\n",
      scheduledInterval,
      intervalReason()
  );
}

void help() {

  Serial.println(
      "Commands: HELP, STATUS, MODE ADAPTIVE, "
      "MODE FIXED, INPUT POT, INPUT TEST"
  );

  Serial.println(
      "MOISTURE 75 (or 50/35/25), TEST WET, "
      "TEST DRY, TEST RECOVER, TEST FAULT"
  );

  Serial.println(
      "FAULT ON, FAULT OFF, RESETDAY, RESET"
  );
}

// --------------------------------------------------
// SERIAL COMMAND HANDLER
// --------------------------------------------------

void command(String s) {

  s.trim();

  String u = s;
  u.toUpperCase();

  if (u == "HELP") {

    help();

  } else if (u == "STATUS") {

    status();

  } else if (u == "MODE ADAPTIVE") {

    mode = ADAPTIVE;

    Serial.println("[MODE] ADAPTIVE");

    scheduleNextRead();

  } else if (u == "MODE FIXED") {

    mode = FIXED;

    Serial.println("[MODE] FIXED_5_MIN");

    scheduleNextRead();

  } else if (u == "INPUT POT") {

    testInput = false;

    Serial.println("[INPUT] Potentiometer");

  } else if (u == "INPUT TEST") {

    testInput = true;

    Serial.println("[INPUT] Serial test input");

  } else if (u.startsWith("MOISTURE ")) {

    testMoisture =
        clampf(u.substring(9).toFloat(), 0, 100);

    testInput = true;

    Serial.printf(
        "[TEST_INPUT] moisture=%.1f%% "
        "equivalent_voltage=%.3f V\n",
        testMoisture,
        DRY_V -
            testMoisture / 100.0f * (DRY_V - WET_V)
    );

  } else if (u == "TEST WET") {

    testInput = true;
    testMoisture = 75;

    Serial.println(
        "[SCENARIO] Wet stable soil 75%."
    );

    acquire();

  } else if (u == "TEST DRY") {

    testInput = true;
    testMoisture = 25;

    Serial.println(
        "[SCENARIO] Persistent dry soil 25%."
    );

    acquire();

  } else if (u == "TEST RECOVER") {

    testInput = true;
    testMoisture = 50;

    Serial.println(
        "[SCENARIO] Moisture recovery 50%."
    );

    acquire();

  } else if (u == "TEST FAULT") {

    forcedFault = true;

    Serial.println(
        "[SCENARIO] Sensor fault injected."
    );

    acquire();

  } else if (u == "FAULT ON") {

    forcedFault = true;

    Serial.println("[TEST] Fault enabled.");

  } else if (u == "FAULT OFF") {

    forcedFault = false;

    Serial.println("[TEST] Fault disabled.");

  } else if (u == "RESETDAY") {

    startsToday = 0;
    dayStart = millis();

    if (state == START_LOCKOUT) {
      state = OFF_READY;
    }

    Serial.println(
        "[TEST] Daily counter reset."
    );

  } else if (u == "RESET") {

    setPump(false, "test_reset");

    state = OFF_READY;
    pending = false;
    validLast = false;

    prevM = NAN;
    lastM = NAN;

    startsToday = 0;
    reads = 0;

    dayStart = millis();
    lastRead = millis();

    Serial.println(
        "[TEST] State/counters reset."
    );

    scheduleNextRead();

  } else {

    Serial.println(
        "[SERIAL] Unknown command; type HELP."
    );
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {

  Serial.begin(115200);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  pinMode(PUMP_LED_PIN, OUTPUT);
  digitalWrite(PUMP_LED_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(
      SENSOR_PIN,
      ADC_11db
  );

  dayStart = millis();
  lastRead = millis();

  Serial.println(
      "\n=== SENTINEL WOKWI PROTOTYPE ==="
  );

  Serial.printf(
      "TIME_SCALE=%.1f; "
      "1 simulated minute = %.2f real seconds\n",
      TIME_SCALE,
      60.0f / TIME_SCALE
  );

  Serial.println(
      "Type HELP in Serial Monitor (115200 baud)."
  );

  scheduleNextRead();
}

// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------

void loop() {

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {

      if (cmd.length()) {

        command(cmd);

        cmd = "";
      }

    } else if (cmd.length() < 100) {

      cmd += c;
    }
  }

  updateDay();

  // Confirmation acquisition has priority
  if (pending &&
      (long)(millis() - confirmDue) >= 0) {

    acquire();

    return;
  }

  // Acquire only when the scheduled interval expires
  if ((long)(millis() - nextReadDue) >= 0) {

    acquire();
  }

  delay(5);
}