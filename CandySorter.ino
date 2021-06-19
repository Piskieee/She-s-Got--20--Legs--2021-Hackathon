#include <LiquidCrystal.h>
#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BH1749NUC_Arduino_Library.h>
#include <SparkFun_PCA9536_Arduino_Library.h>

Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();
#define HOPPER 210
#define SENSOR 320
#define DROP 440
#define BINRED 130
#define BINORANGE 180
#define BINYELLOW 255
#define BINGREEN 325
#define BINBLUE 395
#define BINBROWN 465
BH1749NUC rgb;
PCA9536 io;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
int photoInturruptSignal = 3;
int slotServo = 0;
int slideServo = 1;
int curSlotPos = HOPPER;
int curSlidePos = 290;
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

  // initiate screen
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("==Init==");
  
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

  lcd.setCursor(0, 0);
  lcd.print("Ready to Sort");
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
        countBlue++;
        lcd.setCursor(0, 0);
        lcd.print("Blue            ");
        lcd.setCursor(0, 1);
        lcd.print(countBlue);
        rotateSlide(BINBLUE);
      }
      else if(G > 20000) {
        countGreen++;
        lcd.setCursor(0, 0);
        lcd.print("Green           ");
        lcd.setCursor(0, 1);
        lcd.print(countGreen);
        rotateSlide(BINGREEN);
      }
      else {
        countBrown++;
        lcd.setCursor(0, 0);
        lcd.print("Brown           ");
        lcd.setCursor(0, 1);
        lcd.print(countBrown);
        rotateSlide(BINBROWN);
      }
    }
    else {
      if(G > 20000) {
        countYellow++;
        lcd.setCursor(0, 0);
        lcd.print("Yellow          ");
        lcd.setCursor(0, 1);
        lcd.print(countYellow);
        rotateSlide(BINYELLOW);
      }
      else if(R > 9500) {
        countOrange++;
        lcd.setCursor(0, 0);
        lcd.print("Orange          ");
        lcd.setCursor(0, 1);
        lcd.print(countOrange);
        rotateSlide(BINORANGE);
      }
      else {
        countRed++;
        lcd.setCursor(0, 0);
        lcd.print("Red             ");
        lcd.setCursor(0, 1);
        lcd.print(countRed);
        rotateSlide(BINRED);
      }
    }

    delay(300);
    rotateSlot(DROP);
    delay(500);
    rotateSlot(HOPPER);
    delay(300);

    //clear screen
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
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
