#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();
#define SERVOMIN  120
#define SERVOMAX  720
BH1749NUC rgb;
PCA9536 io; 
int photoInturruptSignal = 3;
int slotServo = 0;
int slideServo = 1;
int curSlotPos = SERVOMIN;
int curSlidePos = SERVOMIN;
int val;
int r, g, b;

void rotateSlot (int target) {
  rotateMotor(slotServo, curSlotPos, target);
  curSlotPos = target;
}

void rotateSlide (int target) {
  rotateMotor(slideServo, curSlidePos, target);
  curSlidePos = target;
}

void rotateMotor (int servo, int starting, int target) {
  if(target < starting) {
    servoController.setPWM(servo, 0, target);
    delay(1000);
  }
  else {
    for (uint16_t pulselen = starting; pulselen < target; pulselen++) {
      servoController.setPWM(servo, 0, pulselen);
    }
    delay(100);  
  }
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  servoController.setPWM(n, 0, pulse);
}

void setup() {
  // enable debug output
  Serial.begin(115200);
  Serial.println("====Setup Initilizing====");
  
  // candy detector
  pinMode(photoInturruptSignal, INPUT); //low is triggered

  // servos
  servoController.begin();
  servoController.setPWMFreq(60);
  servoController.setPWM(slotServo,0,SERVOMIN);
  servoController.setPWM(slideServo,0,SERVOMIN);
  delay(1000);

  // color sensor
  if (rgb.begin() != BH1749NUC_SUCCESS)
  {
    Serial.println("Error initializing the rgb sensor - Halting");
    while (1) ;
  }
  rgb.setRGBGain(BH1749NUC_GAIN_X1);
  rgb.setMeasurementMode(BH1749NUC_MEASUREMENT_MODE_240_MS);

  // color sensor onboard LED
  if (io.begin() == false)
  {
    Serial.println("PCA9536 not detected - Halting");
    while (1);
  }
  
  for(uint16_t pinNum = 0; pinNum < 4; pinNum++)
  {
    io.pinMode(pinNum, OUTPUT);
    io.write(pinNum, HIGH);
  }

  Serial.println("====Candy Sorter Initialized====");
}

void loop() {
  val = digitalRead(photoInturruptSignal);

  if(val == LOW) // beam inturrupt
  {
    Serial.println("dropped");
    delay(500);

    rotateSlot(SERVOMAX);
    rotateSlide(SERVOMAX);
  
    if (rgb.available())
    {
      io.write(0, LOW);
      delay(500);
      val = 3;
  
      Serial.println("Red: " + String(rgb.readRed()));
      Serial.println("Green: " + String(rgb.readGreen()));
      Serial.println("Blue: " + String(rgb.readBlue()));
      Serial.println();
      io.write(0, HIGH);
    }

    rotateSlot(SERVOMIN);
    rotateSlide(SERVOMIN);
  }
}
