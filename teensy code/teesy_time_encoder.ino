#include <math.h>

const int ESC_PIN = 3;
const int ENC_PIN = 4;

// ----------------------------
// Encoder state
// ----------------------------

float lastAngleDeg = 0.0f;
float rps = 0.0f;

unsigned long lastWrapTime = 0;
unsigned long lastAngleUpdate = 0;

bool firstWrap = true;

// ----------------------------
// OneShot125
// ----------------------------

void sendOneShot125(uint16_t pulse_us)
{
    digitalWriteFast(ESC_PIN, HIGH);
    delayMicroseconds(pulse_us);
    digitalWriteFast(ESC_PIN, LOW);
//4kHz
    delayMicroseconds(250 - pulse_us);
}

// ----------------------------
// Read AS5048 PWM
// ----------------------------

float readEncoderAngle()
{
    unsigned long highTime =
        pulseIn(ENC_PIN, HIGH, 10000);

    unsigned long lowTime =
        pulseIn(ENC_PIN, LOW, 10000);

    unsigned long period =
        highTime + lowTime;

    if(period == 0)
        return lastAngleDeg;

    // convert PWM frame to angle

    float highClocks =
        (float)highTime * 4119.0f /
        (float)period;

    float count =
        highClocks - 16.0f;

    if(count < 0)
        count = 0;

    if(count > 4095)
        count = 4095;

    return count * 360.0f / 4095.0f;
}

// ----------------------------
// Update RPS estimate
// ----------------------------

void updateEncoder()
{
    float angle = readEncoderAngle();

    if(lastAngleDeg > 300 &&
       angle < 60)
    {
        unsigned long now = micros();

        if(!firstWrap)
        {
            float revTime =
                (now - lastWrapTime) *
                1e-6f;

            if(revTime > 0)
                rps = 1.0f / revTime;
        }

        firstWrap = false;
        lastWrapTime = now;
    }

    lastAngleDeg = angle;
    lastAngleUpdate = micros();
}

// ----------------------------
// Angle predictor
// ----------------------------

float predictedAngle()
{
    float dt =(micros() - lastAngleUpdate)* 1e-6f;

    float angle =lastAngleDeg +  rps * 360.0f * dt;

    angle = fmodf(angle, 360.0f);

    if(angle < 0)
        angle += 360.0f;

    return angle;
}

// ----------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(ESC_PIN, OUTPUT);
    pinMode(ENC_PIN, INPUT);

    digitalWriteFast(ESC_PIN, LOW);

    // arm ESC

    for(int i = 0; i < 5000; i++)
    {
        sendOneShot125(125);
    }
}

// ----------------------------

void loop()
{
   

    updateEncoder();

    float psi =
        predictedAngle() *
        DEG_TO_RAD;

    uint16_t pulse = 165 + 0 * sinf(psi);

    sendOneShot125(pulse);

    static elapsedMillis printTimer;

    if(printTimer > 100)
    {
        printTimer = 0;

        Serial.print("Angle: ");
        Serial.print(predictedAngle());

        Serial.print("  RPS: ");
        Serial.println(rps);
    }
}