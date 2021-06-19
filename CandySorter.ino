#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();
#define HOPPER 210
#define SENSOR 320
#define DROP 440
BH1749NUC rgb;
PCA9536 io; 
int photoInturruptSignal = 3;
int slotServo = 0;
int slideServo = 1;
int curSlotPos = HOPPER;
int curSlidePos = SERVOMIN;
int loadCount = 0;
int val;
int R, G, B;

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
  servoController.setPWM(slotServo,0,curSlotPos);
  servoController.setPWM(slideServo,0,SERVOMIN);
  delay(500);

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

void run()
{
    rotateSlot(SENSOR);
    delay(300);
        
    if (rgb.available())
    {
      io.write(0, LOW);
      delay(500);
      val = 3;

      R = rgb.readRed();
      G = rgb.readGreen();
      B = rgb.readBlue();
  
      Serial.println("Red: " + String(R));
      Serial.println("Green: " + String(G));
      Serial.println("Blue: " + String(B));
      Serial.println();
      io.write(0, HIGH);
      delay(100);
    }

    if(R < 8000) {
      if(B > 9500) {
        Serial.println("Seems Blue");
        Serial.println();
      }
      else if(G > 20000) {
        Serial.println("Seems Green");
        Serial.println();
      }
      else { 
        Serial.println("Seems Brown");
        Serial.println();
      }
    }
    else {
      if(G > 20000) {
        Serial.println("Seems Yellow");
        Serial.println();
      }
      else if(R > 9500) {
        Serial.println("Seems Orange");
        Serial.println();
      }
      else {
        Serial.println("Seems Red");
        Serial.println();
      }
    }

    delay(300);
    rotateSlot(DROP);
    delay(1000);
    rotateSlot(HOPPER);
    delay(300);
}

void loop() {
  val = digitalRead(photoInturruptSignal);

  if(val == LOW) // beam inturrupt
  {
    run();
    loadCount = 2;
  }
  else if(loadCount > 0)
  {
    run();
    loadCount--;
  }
}
