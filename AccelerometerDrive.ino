#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include "Wire.h"
#include "SimpleTimer.h"

// All global define
#define PRINT_TO_LCD    1
#define NUM_BUTTONS	40
#define NUM_AXES	8	       // 8 axes, X, Y, Z, etc
#define X               0
#define Y               1
#define LCD_ROWS        4
#define LCD_COLUMNS     20
#define OFF_SWITCH_PIN  6
#define X_POT_PIN       A2
#define Y_POT_PIN       A3
#define DEADMAN_PIN     7
#define LCD_I2C_ADDR    0x27

#ifdef PRINT_TO_LCD
#include "LiquidCrystal_I2C.h"
#endif

typedef struct joyReport_t {
	int16_t axis[NUM_AXES];
	uint8_t button[(NUM_BUTTONS+7)/8]; // 8 buttons per byte
} joyReport_t;

joyReport_t joyReport;

MPU6050 myMPU;
#ifdef PRINT_TO_LCD
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

int16_t ax, ay, az;
int16_t gx, gy, gz;
float pitch, roll;
float joySpeedX, joySpeedY;
float lcdSpeedX, lcdSpeedY;

int pauseState = 0;
int deadmanState = 0;
float yReductionFactor, xReductionFactor;

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  myMPU.initialize();
  setupPins();
  
  #ifdef PRINT_TO_LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.setCursor(0,0);
  #endif
  
  delay(200);
  for (uint8_t ind=0; ind<8; ind++) {
    joyReport.axis[ind] = 0;
  }

  for (uint8_t ind=0; ind<sizeof(joyReport.button); ind++) {
    joyReport.button[ind] = 0;
  }
}

// Send an HID report to the USB interface
void sendJoyReport(struct joyReport_t *report)
{
	Serial.write((uint8_t *)report, sizeof(joyReport_t));
}

  // turn a button on
  void setButton(joyReport_t *joy, uint8_t button)
  {
  	uint8_t index = button/8;
  	uint8_t bit = button - 8*index;
  
   	joy->button[index] |= 1 << bit;
  }
  
  // turn a button off
  void clearButton(joyReport_t *joy, uint8_t button)
  {
  	uint8_t index = button/8;
  	uint8_t bit = button - 8*index;
  
  	joy->button[index] &= ~(1 << bit);
  }

void setupPins(void) {
  pinMode(A4, INPUT);
  digitalWrite(A4, HIGH);
  pinMode(A5, INPUT);
  digitalWrite(A5, HIGH);
  pinMode(OFF_SWITCH_PIN, INPUT);
  pinMode(DEADMAN_PIN, INPUT);
}

void loop() 
{
  pauseState = digitalRead(OFF_SWITCH_PIN);
  deadmanState = digitalRead(DEADMAN_PIN);
        
  yReductionFactor = analogRead(Y_POT_PIN);
  xReductionFactor = analogRead(X_POT_PIN);
  myMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));
  roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)));
  
  if(deadmanState == LOW) {
    if(pauseState == HIGH) {
      lcdSpeedX = (pitch/(PI/2)) * (xReductionFactor / 1024);
      lcdSpeedY = (roll/(PI/2)) * (yReductionFactor / 1024);
      joySpeedX = 32768 * (pitch/(PI/2));
      joySpeedY = 32768 * (roll/(PI/2));
      for (uint8_t ind=0; ind<8; ind++) {
        joyReport.axis[ind] = 0;
      }
      
      for (uint8_t ind=0; ind<sizeof(joyReport.button); ind++) {
        joyReport.button[ind] = 0;
      }
      joyReport.axis[X] = joySpeedX * (xReductionFactor / 1024);
      joyReport.axis[Y] = -joySpeedY * (yReductionFactor / 1024);
      
      printToLCD();
    }
    else {
     joyReport.axis[X] = 0;
     joyReport.axis[Y] = 0;
     printDisabledMsg();
    }
  }
  else {
    joyReport.axis[X] = 0;
    joyReport.axis[Y] = 0;
    printDisabledMsg();
  }
  printHeader();  
  
  sendJoyReport(&joyReport);
  
  delay(100);
}

void printToLCD() {
  lcd.clear();
  lcd.setCursor(0,2);
  lcd.print("X: "); lcd.print(lcdSpeedX);
  lcd.setCursor(0,3);
  lcd.print("Y: "); lcd.print(lcdSpeedY);
}

void printHeader() {
    lcd.home();
    lcd.print("NR Custom Joystick!");
}

void printDisabledMsg() {
  lcd.clear();
  lcd.setCursor(0,2);
  lcd.print("Controller paused!");
  lcd.setCursor(0,3);
  lcd.print("Flip ON to unpause");
}
