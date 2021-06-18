#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>

#define SERVOMIN  150;
#define SERVOMAX  600;

Adafruit_PWMServoDriver servoDropper = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver servoSlide = Adafruit_PWMServoDriver();
int photoInturruptSignal = 3;
int slotServo = 0;
int val;

void setup() {
  // candy detector
  pinMode(photoInturruptSignal, INPUT); //low is triggered

  // servos
  servoDropper.begin();
  servoDropper.setPWMFreq(60);
  servoSlide.begin();
  servoSlide.setPWMFreq(60);

  // enable debug output
  Serial.begin(9600);
  Serial.println("====Candy Sorter Initialized====");
}

void loop() {
  val = digitalRead(photoInturruptSignal);

  if(val == LOW) // beam inturrupt
  {
    Serial.println("dropped");
    delay(200);
  }
}
