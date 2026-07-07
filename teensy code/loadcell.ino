// #include <HX711.h>

// #define HX_DT   23
// #define HX_SCK  22

// HX711 scale;

// void setup()
// {
//   Serial.begin(115200);

//   scale.begin(HX_DT, HX_SCK);

//   Serial.println("Remove all weight");
//   delay(5000);

//   scale.tare();

//   Serial.println("Place known weight");
//   delay(5000);

//   long reading = scale.read_average(20);

//   Serial.print("Raw reading = ");
//   Serial.println(reading);
// }

// void loop() {}







#include <HX711.h>

#define HX_DT   2
#define HX_SCK  3

HX711 scale;

// Replace after calibration
float calibration_factor = 432;

void setup()
{
  Serial.begin(115200);

  scale.begin(HX_DT, HX_SCK);

  Serial.println("Taring...");
  scale.tare();   // Zero the scale

  Serial.println("Ready");
}

void loop()
{
  if (scale.is_ready())
  {
    float raw = scale.read_average(2);

    float weight_g = (raw - scale.get_offset()) /
                     calibration_factor;

    Serial.print("Weight: ");
    Serial.print(weight_g);
    Serial.println(" g");
  }
  else
  {
    Serial.println("HX711 not found");
  }

  delay(400);
}