#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();
#define SERVOMIN  120
#define SERVOMAX  720
int photoInturruptSignal = 3;
int slotServo = 0;
int slideServo = 1;
int val;

void setup() {
  // candy detector
  pinMode(photoInturruptSignal, INPUT); //low is triggered

  // servos
  servoController.begin();
  servoController.setPWMFreq(60);

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
/*
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      servoController.setPWM(slotServo, 0, pulselen);
      servoController.setPWM(slideServo, 0, pulselen);
    }

    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      servoController.setPWM(slotServo, 0, pulselen);
      servoController.setPWM(slideServo, 0, pulselen);
    }
    delay(500);
   */

   
  }
}
