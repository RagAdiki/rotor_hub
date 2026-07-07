#include <Servo.h>

Servo esc;

void setup() {
    esc.attach(5);   
    esc.writeMicroseconds(1200);

}

void loop() {

}