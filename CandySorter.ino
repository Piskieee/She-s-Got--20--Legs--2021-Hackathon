#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>

int photoInturruptSignal = 3;
int drop;

void setup() {
  // candy detector
  pinMode(photoInturruptSignal, INPUT); //low is triggered
  
  // enable debug output
  Serial.begin(9600);
}

void loop() {
  drop = digitalRead(photoInturruptSignal);

  if(drop == LOW) // beam inturrupt
  {
    Serial.println("dropped");
    delay(100);
  }
}
