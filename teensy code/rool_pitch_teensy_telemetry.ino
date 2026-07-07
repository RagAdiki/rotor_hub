// ---------- Pin definitions ----------
#define CS_PIN   10
#define CLK_PIN  13
#define MOSI_PIN 11
#define MISO_PIN 12
const int ESC_PIN = 3;

// ---------- Receiver pins ----------
#define PITCH_PIN    5
#define ROLL_PIN     6
#define THROTTLE_PIN 7

#define PWM_MIN     1000
#define PWM_CENTER  1500
#define PWM_MAX     2000

// ---------- Non-blocking receiver capture ----------
volatile uint16_t pitchPWM_v    = 1500;
volatile uint16_t rollPWM_v     = 1500;
volatile uint16_t throttlePWM_v = 1500;

volatile uint32_t pitchStart = 0, rollStart = 0, throttleStart = 0;

void pitchISR()
{
    if (digitalReadFast(PITCH_PIN)) {
        pitchStart = micros();
    } else {
        pitchPWM_v = micros() - pitchStart;
    }
}

void rollISR()
{
    if (digitalReadFast(ROLL_PIN)) {
        rollStart = micros();
    } else {
        rollPWM_v = micros() - rollStart;
    }
}

void throttleISR()
{
    if (digitalReadFast(THROTTLE_PIN)) {
        throttleStart = micros();
    } else {
        throttlePWM_v = micros() - throttleStart;
    }
}

// ---------- RPM tracking ----------
float lastAngle = 0;
unsigned long lastTime = 0;

void sendOneShot125(uint16_t pulse_us)
{
    digitalWriteFast(ESC_PIN, HIGH);
    delayMicroseconds(pulse_us);
    digitalWriteFast(ESC_PIN, LOW);
    delayMicroseconds(500 - pulse_us); // 2 kHz frame
}

void setup()
{
    Serial.begin(115200);

    pinMode(CS_PIN, OUTPUT);
    pinMode(CLK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);

    digitalWrite(CS_PIN, HIGH);
    digitalWrite(CLK_PIN, LOW);
    digitalWrite(MOSI_PIN, HIGH);

    pinMode(ESC_PIN, OUTPUT);
    digitalWriteFast(ESC_PIN, LOW);

    pinMode(PITCH_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(THROTTLE_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(PITCH_PIN),    pitchISR,    CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROLL_PIN),     rollISR,     CHANGE);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);

    // Arm ESC
    for (int i = 0; i < 5000; i++)
    {
        sendOneShot125(125);
    }

    lastTime = micros();
}

uint16_t spiTransfer16(uint16_t tx)
{
    uint16_t rx = 0;
    digitalWrite(CS_PIN, LOW);

    for (int i = 15; i >= 0; i--)
    {
        digitalWrite(MOSI_PIN, (tx >> i) & 1);
        digitalWrite(CLK_PIN, HIGH);
        rx <<= 1;
        if (digitalRead(MISO_PIN))
            rx |= 1;
        digitalWrite(CLK_PIN, LOW);
    }

    digitalWrite(CS_PIN, HIGH);
    return rx;
}

float readRawAngle()
{
    spiTransfer16(0xFFFF);
    delayMicroseconds(1);
    uint16_t data = spiTransfer16(0xFFFF);
    uint16_t raw = data & 0x3FFF;
    return raw * 360.0f / 16384.0f;
}

void loop()
{
    unsigned long t = micros();
    float angle = readRawAngle();

    // ---------------- RPM calculation ----------------
    float deltaAngle = angle - lastAngle;
    const float WRAP_THRESHOLD = -180.0f;
    if (deltaAngle < WRAP_THRESHOLD) {
        deltaAngle += 360.0f;
    } else if (deltaAngle < 0.0f) {
        deltaAngle = 0.0f;
    }

    float deltaTime = (t - lastTime) / 1000000.0f;
    float rpm = 0;
    if (deltaTime > 0) {
        rpm = (deltaAngle / 360.0f) * (60.0f / deltaTime);
    }

    lastAngle = angle;
    lastTime = t;
    // ---------------------------------------------------

    // ---------------- Receiver read (non-blocking) ----------------
    noInterrupts();
    uint16_t pitchPWM    = pitchPWM_v;
    uint16_t rollPWM     = rollPWM_v;
    uint16_t throttlePWM = throttlePWM_v;
    interrupts();

    float p = (float)pitchPWM - PWM_CENTER;
    float r = (float)rollPWM  - PWM_CENTER;

    float stickAngle     = atan2(p, -r) * 180.0 / PI;
    float stickAmplitude = sqrt(p * p + r * r) / 500.0;

    int throttle = map(throttlePWM, PWM_MIN, PWM_MAX, 125, 250);
    throttle = constrain(throttle, 125, 250);
    // -----------------------------------------------------------

    // ---------------- Phase correction ----------------
    angle = angle + 13 -5 + 60 + 20 + +25  ;
    if (angle > 360) {
        angle = angle - 360;
    }

    float psi = (angle + stickAngle) * DEG_TO_RAD;
    float cyclicAmp = throttle * stickAmplitude * 0.1;

    float pulse = throttle + cyclicAmp * sinf(psi);
    pulse = constrain(pulse, 125, 250);

    sendOneShot125((uint16_t)pulse);

    Serial.print("Angle: "); Serial.print(angle, 3);
    Serial.print("  RPM: "); Serial.print(rpm, 2);
    Serial.print("  Throttle: "); Serial.print(throttle);
    Serial.print("  StickAngle: "); Serial.print(stickAngle);
    Serial.print("  Amplitude: "); Serial.println(stickAmplitude);
}