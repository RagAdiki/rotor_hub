

#include <math.h>

const int ESC_PIN = 3;
const int ENC_PIN = 4;

float lastAngle = 0.0f;
unsigned long lastTime = 0;

float rpm = 0.0f;

//--------------------------------------------------
// OneShot125
//--------------------------------------------------

void sendOneShot125(uint16_t pulse_us)
{
    digitalWriteFast(ESC_PIN, HIGH);
    delayMicroseconds(pulse_us);
    digitalWriteFast(ESC_PIN, LOW);

    // 2 kHz frame
    delayMicroseconds(500 - pulse_us);
}

//--------------------------------------------------
// Read AS5048 PWM Angle
//--------------------------------------------------

float readEncoderAngle()
{
    unsigned long highTime = pulseIn(ENC_PIN, HIGH, 10000);
    unsigned long lowTime  = pulseIn(ENC_PIN, LOW, 10000);

    unsigned long period = highTime + lowTime;

    if (period == 0)
        return lastAngle;

    float highClocks = (float)highTime * 4119.0f / (float)period;

    float count = highClocks - 16.0f;

    if (count < 0)
        count = 0;

    if (count > 4095)
        count = 4095;

    return count * 360.0f / 4095.0f;
}

//--------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(ESC_PIN, OUTPUT);
    pinMode(ENC_PIN, INPUT);

    digitalWriteFast(ESC_PIN, LOW);

    // Arm ESC
    for (int i = 0; i < 5000; i++)
    {
        sendOneShot125(125);
    }

    lastAngle = readEncoderAngle();
    lastTime = micros();
}

//--------------------------------------------------

void loop()
{
    // Read encoder
    float angle = readEncoderAngle();

    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6f;

    if (dt > 0.0f)
    {
        float dTheta = angle - lastAngle;

        // Ha

        if (dTheta < -5)
            dTheta += 360.0f;

        float degPerSec = dTheta / dt;

        rpm = degPerSec * 60.0f / 360.0f;
    }

    lastAngle = angle;
    lastTime = now;

    // Sinusoidal command
    float psi = (angle )* DEG_TO_RAD;

    float pulse = 125 + 0* sinf(psi);   // Change amplitude here

    sendOneShot125(pulse);

    // Print every 100 ms
    static elapsedMillis printTimer;

    if (printTimer > 100)
    {
        printTimer = 0;

        Serial.print("Angle: ");
        Serial.print(angle, 2);

        Serial.print("  RPM: ");
        Serial.println(rpm, 2);
    }
}