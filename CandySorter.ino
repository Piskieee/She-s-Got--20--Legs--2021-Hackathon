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
#define BLUE 3
#define GREEN 5
#define RED 6
BH1749NUC rgb;
PCA9536 io;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
int photoInturruptSignal = 4;
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
  Serial.begin(9600);
  //Serial.println("====Setup Initilizing====");

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
  //Serial.println("====Candy Sorter Initialized====");

  //RGB LED
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  analogWrite(RED, 0);
  analogWrite(GREEN, 0);
  analogWrite(BLUE, 0);
}

void printCountsToSerial()
{
  Serial.print(countRed);
  Serial.print(" ");
  Serial.print(countOrange);
  Serial.print(" ");
  Serial.print(countYellow);
  Serial.print(" ");
  Serial.print(countGreen);
  Serial.print(" ");
  Serial.print(countBlue);
  Serial.print(" ");
  Serial.println(countBrown);
}

void rgbOn(int redValue, int greenValue, int blueValue) {
  /*if(redValue > 0) {
    digitalWrite(RED, HIGH);
  }
  if(blueValue > 0) {
    digitalWrite(BLUE, HIGH);
  }
  if(greenValue > 0) {
    digitalWrite(GREEN, HIGH);
  }*/
  analogWrite(RED, redValue);
  analogWrite(BLUE, blueValue);
  analogWrite(GREEN, greenValue);
}

void rgbOff() {
  analogWrite(RED, 0);
  analogWrite(BLUE, 0);
  analogWrite(GREEN, 0);
  //digitalWrite(RED, LOW);
  //digitalWrite(GREEN, LOW);
  //digitalWrite(BLUE, LOW);
}

void run()
{
    rotateSlot(SENSOR);
    delay(100);
        
    if (rgb.available())
    {
      io.write(0, LOW);
      delay(250);


      R = rgb.readRed();
      G = rgb.readGreen();
      B = rgb.readBlue();
  /*
      Serial.println("Red: " + String(R));
      Serial.println("Green: " + String(G));
      Serial.println("Blue: " + String(B));
      Serial.println();
      */
      io.write(0, HIGH);
//      delay(100);
    }

    if(R < 8000) {
      if(B > 9500) {
        countBlue++;
        rgbOn(0,0,255);
        lcd.setCursor(0, 0);
        lcd.print("Blue            ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countBlue));
        rotateSlide(BINBLUE);
      }
      else if(G > 20000) {
        countGreen++;
        rgbOn(0,255,0);
        lcd.setCursor(0, 0);
        lcd.print("Green           ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countGreen));
        rotateSlide(BINGREEN);
      }
      else {
        countBrown++;
        rgbOn(165,42,42);
        lcd.setCursor(0, 0);
        lcd.print("Brown           ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countBrown));
        rotateSlide(BINBROWN);
      }
    }
    else {
      if(G > 22500) {
        countYellow++;
        rgbOn(255,165,0);
        lcd.setCursor(0, 0);
        lcd.print("Yellow          ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countYellow));
        rotateSlide(BINYELLOW);
      }
      else if(R > 9500) {
        countOrange++;
        rgbOn(255,50,0);
        lcd.setCursor(0, 0);
        lcd.print("Orange          ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countOrange));
        rotateSlide(BINORANGE);
      }
      else {
        countRed++;
        rgbOn(255,0,0);
        lcd.setCursor(0, 0);
        lcd.print("Red             ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print(String(countRed));
        rotateSlide(BINRED);
      }
    }

    delay(200);
    rotateSlot(DROP);
    delay(200);
    rotateSlot(HOPPER);
    delay(200);

    rgbOff();
    lcd.setCursor(0, 0);
    lcd.print("Total           ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(countRed+countOrange+countYellow+countGreen+countBlue+countBrown);

    printCountsToSerial();
}

void loop() {
  val = digitalRead(photoInturruptSignal);

  if(val == LOW) // beam inturrupt
  {
    delay(300);
    run();
    loadCount = 2;
  }
  else if(loadCount > 0)
  {
    run();
    loadCount--;
  }
}
