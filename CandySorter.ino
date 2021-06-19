#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();
#define HOPPER 210
#define SENSOR 320
#define DROP 440
#define BINRED 120
#define BINORANGE 150
#define BINYELLOW 180
#define BINGREEN 210
#define BINBLUE 240
#define BINBROWN 270
BH1749NUC rgb;
PCA9536 io; 
int photoInturruptSignal = 3;
int slotServo = 0;
int slideServo = 1;
int curSlotPos = HOPPER;
int curSlidePos = BINRED;
int loadCount = 0;
int val;
int R, G, B;
int countRed = 0;
int countOrange = 0;
int countYellow = 0;
int countGreen = 0;
int countBlue = 0;
int countBrown = 0;

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
  servoController.setPWM(slideServo,0,curSlidePos);
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
        rotateSlide(BINBLUE);
        countBlue++;
      }
      else if(G > 20000) {
        rotateSlide(BINGREEN);
        countGreen++;
      }
      else {
        rotateSlide(BINBROWN);
        countBrown++;
      }
    }
    else {
      if(G > 20000) {
        rotateSlide(BINYELLOW);
        countYellow++;
      }
      else if(R > 9500) {
        rotateSlide(BINORANGE);
        countOrange++;
      }
      else {
        rotateSlide(BINRED);
        countRed++;
      }
    }

    delay(300);
    rotateSlot(DROP);
    delay(500);
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
