#include <Servo.h>

//Change the below values (ensure pwm is between 1000 and 2000 though)
byte servoPin = 11;
float pwm_center = 1150;
float pwm_amplitude = 0 ;
float psi =120 ;//try out values for omega

Servo servo;

float PWM_input = 1320;
uint32_t t_start = 0;


void setup(){
  Serial.begin(460800);
  servo.attach(servoPin);
  
  // ESC stop
  servo.writeMicroseconds(1000); // send "stop" signal to ESC.
   // time to let the ESC recognise and stop 
  t_start = micros();
}


void loop() {
  // put your main code here, to run repeatedly:
  
  uint32_t t_now = micros();
  float t = (t_now - t_start)/1000000.0; //time in seconds
  float pwm = pwm_center + pwm_amplitude*sin(psi*t); 

while (t_now >=15000000) {
   pwm=1000;
   servo.writeMicroseconds(pwm);
 }

  
  servo.writeMicroseconds(pwm); // Send signal to ESC.
  Serial.print(pwm);
  Serial.print("  ");
  Serial.print(t_now);
  Serial.print("\n");
}