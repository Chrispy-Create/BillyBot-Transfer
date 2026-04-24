/**
 * BillyBot — Arduino Mega ultrasonic sensor board
 *
 * 12 × HC-SR04 sensors, 3 height levels, 4 sensors per level.
 * All sensors face roughly forward/rearward at slight angles.
 *
 * Layout (index 0-11):
 *   LOW  level : [0] left  [1] center-left  [2] center-right  [3] right
 *   MID  level : [4] left  [5] center-left  [6] center-right  [7] right
 *   HIGH level : [8] left  [9] center-left [10] center-right [11] right
 *
 * Pin assignments (change to match your wiring):
 *   Sensor i  →  TRIG = TRIG_PINS[i],  ECHO = ECHO_PINS[i]
 *
 * Serial output (115200 baud, USB to Pi):
 *   US:d0:d1:d2:d3:d4:d5:d6:d7:d8:d9:d10:d11\r\n
 *   where d0-d11 are distances in metres (3 decimal places).
 *   4.50 is sent when a sensor reads out-of-range (no echo).
 *
 * Scan rate: ~5 Hz (sequential triggering avoids acoustic crosstalk)
 */

static const int NUM_SENSORS = 12;

// ── Pin assignments ──────────────────────────────────────────────────────────
// Using even/odd pairs on Mega digital pins 22-45 for easy wiring.
static const int TRIG_PINS[NUM_SENSORS] = {
    22, 24, 26, 28,   // LOW  : L, CL, CR, R
    30, 32, 34, 36,   // MID  : L, CL, CR, R
    38, 40, 42, 44    // HIGH : L, CL, CR, R
};
static const int ECHO_PINS[NUM_SENSORS] = {
    23, 25, 27, 29,
    31, 33, 35, 37,
    39, 41, 43, 45
};

// HC-SR04 max reliable range ~4 m; 23 200 µs ≈ 4 m round-trip at 343 m/s.
static const unsigned long ECHO_TIMEOUT_US = 23200UL;

// Delay between triggering consecutive sensors (ms).
// Prevents one sensor's echo being picked up by the next sensor.
static const int INTER_SENSOR_DELAY_MS = 5;

// ── Helpers ──────────────────────────────────────────────────────────────────

/**
 * Trigger one HC-SR04 and return the measured distance in metres.
 * Returns 4.50 if no echo is received within ECHO_TIMEOUT_US.
 */
float measureDistance(int trigPin, int echoPin) {
    // Ensure trig is low before pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(4);

    // 10 µs HIGH pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo pulse width (µs)
    unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

    if (duration == 0) {
        return 4.50f;   // out of range / no echo
    }

    // distance (m) = duration (µs) × speed_of_sound / 2
    //              = duration × 343e-6 / 2
    //              = duration × 0.0001715
    return (float)duration * 0.0001715f;
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }

    // Brief settle time after power-on
    delay(500);
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void loop() {
    float distances[NUM_SENSORS];

    // Read sensors sequentially with a small gap between each
    for (int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = measureDistance(TRIG_PINS[i], ECHO_PINS[i]);
        delay(INTER_SENSOR_DELAY_MS);
    }

    // Send packet:  US:d0:d1:...:d11\r\n
    Serial.print("US:");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(distances[i], 3);
        if (i < NUM_SENSORS - 1) Serial.print(':');
    }
    Serial.println();  // \r\n
}
