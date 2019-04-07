/*

  SleepyClock for watchX v1.2, build with Atom & PlatformIO
  See ReadMe.txt in Project Folder and platformio.ini file

*/

// Version
#define PROGRAM "SleepyClock v0.1.9"

#include <Adafruit_SleepyDog.h>
#include <MPU6050.h>               // MPU Library including Interrupt functions
#include "SSD1306Ascii.h"          // OLED Lib Main Part
#include "SSD1306AsciiSpi.h"       // OLED Lib SPI Part
#include <DS3232RTC.h>             // Alternative RTC Lib with Interrupt Support https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>             // Very usefull library, can be used for someting like oled << "Text" << variable << "Text2"
#include <SparkFun_MAG3110.h>      // Needed for MAG3110
#include <JC_Button.h>             // Needed for some Button Operations
#include <EEPROM.h>                // Needed for accessing the EEPROM
#include <Adafruit_BMP280.h>       // Needed for BMP280

// Libraries placed in "lib"
#include "Edge.h"                  // Simple positive/negative flanc detection based on bounce2

// header files in "include"
// Blockfont file
#include "Clock5x7.h"              // You need to have the file available in the Sketch or Library folders
// MPU Calibration Values
#include "MPU_CalVal_watchX.h"     // Calibration Values for my watchX
//#include "MPU_CalVal_Breakout.h"   // Needed for test enviroment

// Objects
// No need to define an RTC Object with DS3232RTC.h and AVR MCU's
SSD1306AsciiSpi oled;
MPU6050         mpu(0x69);         // Chip's AD0 = 1
MAG3110         mag = MAG3110();
Adafruit_BMP280 bme; // I2C
EDGE            readClockPosEdge, blink100edge, blink250edge, blink500edge;
//EDGE            b1edge, b2edge, b3edge, readClockPosEdge;

// "defines"
//Debug or not Debug...
#define DEBUG 0
//#define DEBUG 1                  // Shows actually additional infos, maybe overwrite other data

#define OLED_DC            A3
#define OLED_CS            A5
#define OLED_RST           A4
#define BLE_CS             A2
#define BLE_IRQ            0
#define BLE_RST            A1
#define BAT_PIN            A11
#define BAT_EN             4
#define CRG_STATE          5        // PullUp, Low = Charging, High = Fully charged
#define BUZZER             9
#define MPU_INT            7        // PullUp
#define RTC_INT            1        // PullUp, Pin 1/ INT3/TX1
#define LEDL               13
#define LEDR               6
#define BUTTON1            8        // PullUp
#define BUTTON2            11       // PullUp
#define BUTTON3            10       // PullUp

#define COMP_OFFSET        8        // Compile and Transfer Time Offset in secs
#define BUT_LONGPRESSED    1000     // Time [ms] for an Button to be "longpressed"

//#define SLEEPTIME           60      // Steps [ms] 8000,4000,2000,1000,500,250,120,60,30,15 see "WatchdogAVR.cpp"
//#define SLEEPTIME          120      // Steps [ms] 8000,4000,2000,1000,500,250,120,60,30,15 see "WatchdogAVR.cpp"
//#define SLEEPTIME          250      // Steps [ms] 8000,4000,2000,1000,500,250,120,60,30,15 see "WatchdogAVR.cpp"
#define SLEEPTIME          500      // Steps [ms] 8000,4000,2000,1000,500,250,120,60,30,15 see "WatchdogAVR.cpp"
#define CLOCKTIME          3500     // "Show the Time" Time
//#define CHECKPOSTIME       5      // Max time for checking Position
//#define CHECKPOSTIME       10      // Max time for checking Position
#define CHECKPOSTIME       20      // Max time for checking Position
//#define CHECKPOSTIME       100      // Max time for checking Position

#define BATTERY_CRITCAL    10       // Battery Critical Level
#define BATTERY_WARNING    15       // Battery Warning Level
#define BATTERY_OK         30       // Battery OK Level

// JC Button Objects
Button          b1btn(BUTTON1);    // BTN connected from pin to ground, 25ms debounce, pullup enabled, logic inverted
Button          b2btn(BUTTON2);
Button          b3btn(BUTTON3);


// Clockpos/Waitpos (Z can be ignored)
const float xclockposmin =  25;
const float xclockposmax =  85;
const float yclockposmin =  -10;
const float yclockposmax =   10;
int16_t ax_prev = 0, ay_prev = 0, az_prev = 0;

// MAG3110
int heading = 0;

// Battery
float voltage;
uint8_t percent;
bool batteryWarning = false;
bool batteryCritical = false;
bool batteryAlarmWarning = false;
bool batteryAlarmCritical = false;

//USB Power Detection
bool usbConnected = false;
bool charging = false;

//ChargeLED(R) Vars
long int chargeledmillis = 0;
const uint8_t chargeledinterval = 50;
const uint8_t chargeledmaxvalue = 255;
const uint8_t chargeledminvalue = 10;
uint8_t chargeledfadevalue = chargeledminvalue;         // !!Important to initialize the startvalue with minvalue!!
uint8_t chargeledfadesteps = 5;
bool LEDL_VAR = false;

// startTime compileTime
time_t startTime, runTime;

// Display Variables
unsigned long previousclockmillis = 0;
bool showClock = false;
bool initClock = true;
bool showStats = false;
bool initStats = true;
bool showSetup = false;
bool initSetup = true;
bool showSensors = false;
bool initSensors = true;
uint16_t shows = 0;
int sleepMS = 0;
unsigned long wakeupmillis = 0;
bool doPosCheck = false;                         // Position Check

//Date & Time
char datebuffer[10];
const char *months[13] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
const char *days[8] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
uint8_t prev_min = 61, prev_sec = 61, prev_hr = 25;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true when loop starts
bool minChanged = false, secChanged = false, hrChanged = false;

// RTC Vars
uint8_t act_hr, act_min, act_sec, act_year, act_month, act_day, act_dayofweek, act_hr12, hr10, hr1, min10, min1, sec2;

//100ms Blinker Vars
long int blink100millis = 0;
const uint8_t blink100interval = 100;
bool blink100 = false;
//bool prev_blink100 = false;
bool blink100_pos = false;  // Flanc

//250ms Blinker Vars
long int blink250millis = 0;
const uint8_t blink250interval = 250;
bool blink250 = false;
//bool prev_blink250 = false;
bool blink250_pos = false;  // Flanc

//500ms Blinker Vars
long int blink500millis = 0;
const int blink500interval = 500;
bool blink500 = false;
//bool prev_blink500 = false;
bool blink500_pos = false;  // Flanc

// Setup
#define       numberOfSetupElements  9               // 9 = 0..8

// Menu Vars, Setup-Element Structure
typedef struct {
  const char *name;
  uint8_t value;
  bool changed;
} setupStruct;

setupStruct setupElement[numberOfSetupElements] = {
    "",0,false,               //  0
    "Exit",0,false,           //  1
    "Save",0,false,           //  2
    "Hour",0,false,           //  3
    "Minute",0,false,         //  4
    "Day",0,false,            //  5
    "Month",0,false,          //  6
    "Year",0,false,           //  7
    "Contrast",0,false        //  8

};

#define       menuStart 1                            // First Menu Value from Array
const uint8_t menuEnd = numberOfSetupElements - 1;   // Last Menu Value from Array
#define       menuValueStart   3                       // First real Menu Entry from Array
const uint8_t menuValueEnd = menuEnd;                // Last real Menu Entry from Array
#define       menuValueExit      1                   // menuValuesX as Numerics
#define       menuValueSave      2
#define       menuValueHour      3
#define       menuValueMinute    4
#define       menuValueDay       5
#define       menuValueMonth     6
#define       menuValueYear      7
#define       menuValueContrast  8
#define       menuEditLine       3                  // Display Line where the values are editable
#define       menuTextPos        6                  // Positin of Menu Value "Text"
#define       menuValuePos       90                 // Positin of Menu Value "Value"

uint8_t menuIndex = 1;                              // Start at 1 (actual Exit)
uint8_t prev_menuIndex = menuIndex;                 // Helper for menuIndex
bool menuIndex_changed = false;
bool menuEditMode  = false;
bool setupEditDone = false;
float xMenu = 0, yMenu = 0;
uint8_t prev_setupElementValue;
bool setupElementValue_changed;

// Button Vars
bool b1pos = false, b2pos = false, b3pos = false;
//bool b1neg = false, b2neg = false, b3neg = false;
bool b2long = false, b3long = false;

//------------------------------------------------------------
//----------------------- Functions --------------------------
//---- For PlatformIO the Functions must be placed on Top ----
//------------------------------------------------------------

// Checkposition
// x,y floats to be checked against the Min & Max values (z don't need to be checked)
bool checkposition(float x, float y, float xmin, float xmax, float ymin, float ymax) {
  if ((x >= xmin) && (x <= xmax) &&
      (y >= ymin) && (y <= ymax)) {
    return true;  // Position is OK
  }
  else {
    return false; // Position is NOK
  }
}

bool getMenuEditMode(uint8_t value) {
  if ((value >= menuValueStart) && (value <= menuValueEnd)) return true;
  else return false;
}

bool getMenuFuncMode(uint8_t value) {
  if ((value == menuValueExit) || (value == menuValueSave)) return true;
  else return false;
}

// Get Battery Values
void getBattery() {
  // Get Battery Values
  digitalWrite(BAT_EN, HIGH);
  delay(50);
  voltage = analogRead(BAT_PIN);
  voltage = (voltage / 1024) * 3.35;
  voltage = voltage / 0.5;
  delay(50);
  digitalWrite(BAT_EN, LOW);
  //Calculate Values
  percent = (voltage - 3.4) / 0.008;
  if (percent > 100){
    percent = 100;
  }
}

// function to return the compile date and time as a time_t value
time_t getCompileTime() {
    const time_t FUDGE(COMP_OFFSET);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[3], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day =    atoi(compDate + 4);
    tm.Year =   atoi(compDate + 7) - 1970;
    tm.Hour =   atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
    //return t;
}

void showBlockDPoint (uint8_t x, uint8_t y) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);

  oled.setCursor(x,y);
  oled.print("1");
  //oled.setCursor(x,y+1);
  //oled.print(" ");
  oled.setCursor(x,y+2);
  oled.print("1");

  // Set Font back
  oled.setFont(currentFont);
}

void showBlockNumber (uint8_t x, uint8_t y, uint8_t value) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);

  switch (value) {
    case 0:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("* .*");
      oled.setCursor(x,y+2);
      oled.print("*./*");
      oled.setCursor(x,y+3);
      oled.print("*/ *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;
    case 1:
      oled.setCursor(x,y);
      oled.print("  (*");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("   *");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("   *");
    break;

    case 2:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("(**,");
      oled.setCursor(x,y+3);
      oled.print("*   ");
      oled.setCursor(x,y+4);
      oled.print("****");
    break;

    case 3:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print(" ***");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;

    case 4:
      oled.setCursor(x,y);
      oled.print("*   ");
      oled.setCursor(x,y+1);
      oled.print("* * ");
      oled.setCursor(x,y+2);
      oled.print("+***");
      oled.setCursor(x,y+3);
      oled.print("  * ");
      oled.setCursor(x,y+4);
      oled.print("  * ");
    break;

    case 5:
      oled.setCursor(x,y);
      oled.print("****");
      oled.setCursor(x,y+1);
      oled.print("*   ");
      oled.setCursor(x,y+2);
      oled.print("***)");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;

    case 6:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*   ");
      oled.setCursor(x,y+2);
      oled.print("***)");
      oled.setCursor(x,y+3);
      oled.print("*  *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;

    case 7:
      // Original 7
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("   *");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("   *");
    break;

    case 8:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*  *");
      oled.setCursor(x,y+2);
      oled.print("****");
      oled.setCursor(x,y+3);
      oled.print("*  *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;

    case 9:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*  *");
      oled.setCursor(x,y+2);
      oled.print("+***");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;
  }
  // Set Font back
  oled.setFont(currentFont);
}

void printSpecialChar(uint8_t x, uint8_t y, uint8_t c) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);

  oled.setCursor(x,y);
  oled.print(char(c));

  // Set Font back
  oled.setFont(currentFont);
}

void showBatteryIcon(uint8_t x, uint8_t y) {
  if ((percent >=  0) && (percent < 25)) printSpecialChar(x,y,35);
  if ((percent >= 25) && (percent < 50)) printSpecialChar(x,y,36);
  if ((percent >= 50) && (percent < 75)) printSpecialChar(x,y,37);
  if ((percent >= 75) && (percent < 100)) printSpecialChar(x,y,38);
  if (percent == 100) printSpecialChar(x,y,39);
}

void showWatchface() {
  if (initClock || minChanged) {
    showBlockNumber (0,   2, hr10);
    showBlockNumber (30,  2, hr1);
    showBlockNumber (73,  2, min10);
    showBlockNumber (103, 2, min1);
  }

  // Double-Point & Battery Symbol blinker
  if (sec2 && secChanged) {
    showBlockDPoint (60, 3);
  }
  if (!sec2 && secChanged) {
    oled.clear(60, 64, 3, 5);
  } //endif sec2

  // Show/Actualize Battery Icon at init or every minute
  if ((!batteryWarning && secChanged) || initClock) {
    showBatteryIcon(0,0);
  }
  // Battery Warning = Blinking Battery Icon
  if (batteryWarning & blink500 && blink250_pos) {
    printSpecialChar(0,0,32);   // Clear Battery Symbol Position
  }
  if (batteryWarning & !blink500 && blink250_pos) {
    showBatteryIcon(0,0);       // Show Battery Symbol
  } //endif sec2

  if (initClock || minChanged) {
    // Show date
    oled.setCursor(24,0);
    oled << act_day << "." << months[act_month] << " " << act_year+2000 << " (" << days[act_dayofweek] << ")";
  }  //endif minChanged

  // Show seconds indicator
  // Build the line
  if (initClock) {
    for (uint8_t i=0; i <act_sec; i++) {
      printSpecialChar(1+i*2,7,50);
    }
  }
  // Clear line at 0 secs
  if ((act_sec == 0) && secChanged) {
    oled.setCursor(0,7);
    oled.clearToEOL();
  }
  // Show sign
  if (initClock || secChanged) {
    printSpecialChar(1+act_sec*2,7,50);
  }

  initClock = false;  //Init View done
}  // End Watchface

// -------------------------------------------------------------------
// ---------------------------- Setup --------------------------------
// -------------------------------------------------------------------

void setup() {

  // Init Display and Clean up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();

  // New RTC lib
  oled.print("RTC");
  // Initialize the Alarms to known values, clear the Alarm flags, clear the Alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  oled.print(".");
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  oled.print(".");
  RTC.alarm(ALARM_1);
  oled.print(".");
  RTC.alarm(ALARM_2);
  oled.print(".");
  RTC.alarmInterrupt(ALARM_1, false);
  oled.print(".");
  RTC.alarmInterrupt(ALARM_2, false);
  oled.print(".");
  RTC.squareWave(SQWAVE_NONE);
  oled.print(".");

  startTime = RTC.get();
  oled.print(".");
  time_t compileTime = getCompileTime();
  if (startTime < compileTime) {
    RTC.set(compileTime);
    oled.print(".");
    startTime = RTC.get();
    oled.print(".");
  }
  oled.println("Ok");

  // MPU
  mpu.initialize();
  mpu.setAccelerometerPowerOnDelay(3);                    // MPU6050_DELAY_3MS
  mpu.setDHPFMode(MPU6050_DHPF_5);                        // MPU6050_DHPF_5HZ
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);        // MPU Gyro_Config Register 1B/27, after init 250°/s
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);        // MPU Accel_Config Register 1C/28, after init 2g
  mpu.setIntFreefallEnabled(false);                       // Disable Freefall Interrupt
  mpu.setIntZeroMotionEnabled(false);                     // Disable Zero Motion Interrupt
  mpu.setIntMotionEnabled(false);                         // Disable Motion Interrupt
  //mpu.setInterruptMode(1);                                // Interrupt Mode; 1= active low / 0 = active high
  //mpu.setInterruptDrive(1);                              // open drain, not needed here !?
  //mpu.setInterruptLatch(1);                               // Interrupt Latch Mode; 1 = signal until int bit manually cleared / 0 = 50us signal

  // Get Offsets from IMU_Zero Sketch
  // See https://github.com/venice1200/TapClock/blob/master/CalibratingSketch/WatchX_IMU_Zero_0x69.ino
  mpu.setXAccelOffset(mpu_XAccelOffset);
  mpu.setYAccelOffset(mpu_YAccelOffset);
  mpu.setZAccelOffset(mpu_ZAccelOffset);
  mpu.setXGyroOffset(mpu_XGyroOffset);
  mpu.setYGyroOffset(mpu_YGyroOffset);
  mpu.setZGyroOffset(mpu_ZGyroOffset);

  //Modify the following for your needs
  //mpu.setMotionDetectionThreshold(80); //80
  //mpu.setMotionDetectionDuration(4);   //4

  // Init Compass MAG3110 and send it directly into standby
  mag.initialize();
  mag.start();                  // Needed ??
  mag.enterStandby();

  // Init BMP280
  bme.begin(118);  // i2c 0x76 at wacthX

  // Hardware
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  // Edge Objects
  //b1edge.init();
  //b2edge.init();
  //b3edge.init();
  readClockPosEdge.init();
  blink100edge.init();
  blink250edge.init();
  blink500edge.init();

  // Button Objects
  b1btn.begin();
  b2btn.begin();
  b3btn.begin();

  // Enable VBUS Pad for detecting USB Power
  USBCON|=(1<<OTGPADE);

  // Get Values from EEPROM
  // Contrast 0-255
  setupElement[menuValueContrast].value = EEPROM.read(5);

  delay(1500);
  oled.clear();
  oled.setCursor(32,0);
  oled.set1X();
  oled.print(F("Welcome to"));
  oled.setCursor(10,7);
  oled << F(PROGRAM);
  oled.setCursor(28,3);
  oled.set2X();
  oled.print("watchX");
  oled.set1X();
  delay(2500);
  oled.clear();
  delay(1000);

}

// -------------------------------------------------------------------
// ------------------------ Main Loop --------------------------------
// -------------------------------------------------------------------

void loop() {

  // main loop variables
  time_t sysTime;
  unsigned long currentmillis = millis();
  tmElements_t rtcAdjustTime;

  // MPU Vars
  int16_t ax=0, ay=0, az=0;
  float arx=0, ary=0, arz=0;
  bool readClockPos = false;

  // Buttons
  b1btn.read();                    // Update Button Object/Read
  b2btn.read();
  b3btn.read();
  b1pos = b1btn.wasPressed();      // Detect if Button was pressed between 2 read's = Pos Flanc
  b2pos = b2btn.wasPressed();
  b3pos = b3btn.wasPressed();
  b2long = b2btn.pressedFor(BUT_LONGPRESSED); //
  b3long = b3btn.pressedFor(BUT_LONGPRESSED);

  sysTime = RTC.get();
  act_hr    = hour(sysTime);
  act_min   = minute(sysTime);
  act_sec   = second(sysTime);
  act_year  = year(sysTime) - 2000;
  act_month = month(sysTime);
  act_day   = day(sysTime);
  act_dayofweek = weekday(sysTime);

  hr10  = act_hr / 10;    // tens of hr
  hr1   = act_hr % 10;    // ones of hr
  min10 = act_min / 10;   // tens of min
  min1  = act_min % 10;   // ones of min
  sec2  = act_sec % 2;    // all 2 secs

  // Runtime
  runTime = sysTime - startTime;

  // Secs, Mins or Hrs changed?
  secChanged = act_sec != prev_sec?true:false;
  minChanged = act_min != prev_min?true:false;
  hrChanged  = act_hr  != prev_hr?true:false;
  // Update prev's
  prev_sec = act_sec;
  prev_min = act_min;
  prev_hr  = act_hr;

  // Wakeup and Position check
  /*
  // V1
  // Enable MPU and read values until you get changed values, means MPU alive
  if (mpu.getSleepEnabled()) mpu.setSleepEnabled(false);
  do {
    mpu.getAcceleration(&ax, &ay, &az);
  } while ((ax == ax_prev) && (ay == ay_prev) && (az == az_prev));
  // Update prev's
  ax_prev = ax;
  ay_prev = ay;
  az_prev = az;
  */

  /*
  // V2
  // Make it simpler and maybe faster
  if (mpu.getSleepEnabled()) {
    mpu.setSleepEnabled(false);        // Enable MPU
    delay(50);                         // Wait a moment bt only if the system has slept before
  }
  mpu.getAcceleration(&ax, &ay, &az);  // GET MPU Values
  */

  // V3
  // Do Position check for max "CHECKPOSTIME"
  // Wakeup MPU if disabled, set Timer with Current Millis and set "doPosCheck" to "1"
  if (mpu.getSleepEnabled()) {
    mpu.setSleepEnabled(false);
    wakeupmillis = currentmillis;
    doPosCheck = true;
  }

  // End check after "CHECKPOSTIME"
  if ( (currentmillis - wakeupmillis) >= CHECKPOSTIME ) {
    doPosCheck = false;
  }

  // Read MPU
  mpu.getAcceleration(&ax, &ay, &az);  // GET MPU Values
  // Calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az)));
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
  // Check Position and Update the Edge Object
  readClockPos = checkposition(arx, ary, xclockposmin, xclockposmax, yclockposmin, yclockposmax);
  readClockPosEdge.update(readClockPos);
  // V3 end

  // LED Fader
  if (currentmillis - chargeledmillis > chargeledinterval) {
    chargeledmillis = currentmillis;
    chargeledfadevalue = chargeledfadevalue + chargeledfadesteps;
    if ((chargeledfadevalue <= chargeledminvalue) || (chargeledfadevalue >= chargeledmaxvalue)) chargeledfadesteps = -chargeledfadesteps;
  }

  // 100ms Blinker
  if (currentmillis - blink100millis > blink100interval) {
    blink100millis = currentmillis;
    blink100 = !blink100;
  }

  // Positive flanc blink100 v2
  //blink100_pos = blink100 != prev_blink100?true:false;
  //prev_blink100 = blink100;

  // 250ms Blinker
  if (currentmillis - blink250millis > blink250interval) {
    blink250millis = currentmillis;
    blink250 = !blink250;
  }

  // Positive flanc blink250 v2
  //blink250_pos = blink250 != prev_blink250?true:false;
  //prev_blink250 = blink250;

  // 500ms Blinker
  if (currentmillis - blink500millis > blink500interval) {
    blink500millis = currentmillis;
    blink500 = !blink500;
  }

  // Positive flanc blink500 v2
  //blink500_pos = blink500 != prev_blink500?true:false;
  //prev_blink500 = blink500;

  // Using EDGE instead of manual flanc programming
  blink100edge.update(blink100);
  blink250edge.update(blink250);
  blink500edge.update(blink500);
  blink100_pos = blink100edge.rising();
  blink250_pos = blink250edge.rising();
  blink500_pos = blink500edge.rising();

   // USB connected?
  usbConnected = (USBSTA&(1<<VBUS));

  // Generate charging bit
  charging = usbConnected && !digitalRead(CRG_STATE);

  // Show charging LEDR
  if (charging) {
    analogWrite(LEDR, chargeledfadevalue);
  }
  else {
    analogWrite(LEDR, 0);
  }

  // LEDL (On for max 100ms)
  //digitalWrite(LEDL, doPosCheck);

  // Sleep
  // If Clock is not shown and no other Mode is active send MPU and MCU to sleep and Power off Display
  // "doPosCheck" prevents the System from Sleep
  // An USB Power connection prevents the System from Sleep
  if (!showClock && !showStats && !showSetup && !showSensors &&!doPosCheck && !usbConnected) {
    initClock = true;                                  // Reset Clock Init
    // Clear Oled and Power Off
    oled.clear();
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    // Send MPU to sleep
    mpu.setSleepEnabled(true);
    // while (!mpu.getSleepEnabled());                 // Wait here, just to be sure MPU returns "I am sleeping".
    // Sleeeeeeeeeeeeep...
    sleepMS = Watchdog.sleep(SLEEPTIME);

    // -----------------------------------------
    // The sketch starts here after sleeping....
    // -----------------------------------------
  }

  // If watchX Position is OK (see vars) show Clock for "CLOCKTIME"
  //if ((readClockPosEdge.rising() && !showClock) || usbConnected) {
  if (readClockPosEdge.rising() && !showClock) {
    showClock = true;
    previousclockmillis = currentmillis;
    mpu.setSleepEnabled(true);
  }

  // If Stats or Setup is active or USB is connected, update the time counter permanently so the CLOCKTIME never runs through completely
  if (showStats || showSetup || showSensors || usbConnected) {
    previousclockmillis = currentmillis;
  }

  // If Time is over and USB is not connected clear "showClock"
  if (showClock && !usbConnected && ((currentmillis - previousclockmillis) >= CLOCKTIME)) {
    showClock = false;
    initClock = true;
  }

  // Stats On
  if (showClock && !showStats && !showSetup && !showSensors && b2pos) {
    showStats = true;
    initStats = true;
    initClock = true;
  }

  // Setup On
  if (showClock && !showSetup && !showStats && !showSensors && b1pos) {
    showSetup = true;
    initSetup = true;
    initClock = true;
  }

  // Sensors On
  if (showClock && !showSensors && !showSetup && !showStats && b3pos) {
    showSensors = true;
    initSensors = true;
    initClock = true;
    mpu.setSleepEnabled(false);
    mag.exitStandby();
  }

  // Stats Off
  if (showStats && !initStats && b1pos) {
    showStats = false;
  }

  // Setup Exit
  if (showSetup && !initSetup && (menuIndex == menuValueExit) && b1pos) {
    showSetup = false;
  }

  // Sensors Exit
  if (showSensors && !initSensors && b1pos) {
    showSensors = false;
    mpu.setSleepEnabled(true);
    mag.enterStandby();
  }

  // Setup Save
  if (showSetup && !initSetup && (menuIndex == menuValueSave) && b1pos) {
    // Save Time to RTC if something's changed
    if ( setupElement[menuValueHour].changed || setupElement[menuValueMinute].changed ||
         setupElement[menuValueDay].changed ||  setupElement[menuValueMonth].changed ||
         setupElement[menuValueYear].changed )
    {
      rtcAdjustTime.Hour = setupElement[menuValueHour].value;
      rtcAdjustTime.Minute = setupElement[menuValueMinute].value;
      rtcAdjustTime.Second = 0;
      rtcAdjustTime.Day = setupElement[menuValueDay].value;
      rtcAdjustTime.Month = setupElement[menuValueMonth].value;
      rtcAdjustTime.Year = setupElement[menuValueYear].value +2000-1970;
      RTC.write(rtcAdjustTime);
    }
    if ( setupElement[menuValueContrast].changed ) {
      EEPROM.write(5,setupElement[menuValueContrast].value);
    }
    showSetup = false;
  }

  // Update Battery
  // If "Clock" is shown update Battery on "init" and then each Minute
  // If "Stats" is shown update Battery each Second
  //if ( (showClock && (minChanged || initClock)) ||
  //     (showStats && secChanged) ) {
  //  getBattery();
  //}

  // Check Battery Level
  // Reset Battery Warning/Critical
  if (percent > BATTERY_OK) {
    batteryWarning = false;
    batteryCritical = false;
  }

  // Battery Warning
  if (percent < BATTERY_WARNING) {
    batteryWarning = true;
  }

  // Battery Critical
  if (percent < BATTERY_CRITCAL) {
    batteryCritical = true;
  }

  // --------------------- Show Clock :-) -------------------
  if (showClock && !showStats && !showSetup && !showSensors) {
    // Power on Display, Re-Init USB and get Battery values
    if (initClock) {
      USBDevice.attach();
      oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      oled.clear();
      shows++;
      getBattery();
    }

    // Get Battery each Minute on Power
    if (minChanged && usbConnected) {
      getBattery();
    }
    // Update Display
    showWatchface();
  } // endif showClock

  // ------------------- Show Stats --------------------
  if (showStats) {
    // Reset Values on seconds keypress
    if (!initStats && b3pos) {
      startTime = RTC.get();
      shows = 0;
    }
    // Do Init of Stats Page
    if (initStats) {
      getBattery();
      oled.clear();
      oled.print(F("<Stats>"));
      oled.setCursor(6,2);
      oled.print(F("Uptime: "));
      oled.setCursor(6,3);
      oled.print("Shows: ");
    } // endif initStats

    // Show Stats Values
    if (secChanged || initStats) {
      getBattery();
      // Show Time
      sprintf(datebuffer, "%02u:%02u:%02u", act_hr, act_min, act_sec);
      oled.setCursor(80,0);
      oled.print(datebuffer);
      //Show Uptime
      sprintf(datebuffer, "%02u:%02u", int(runTime/3600), int((runTime/60)%60));
      oled.setCursor(54,2);
      oled.print(datebuffer);
      // Show counter
      oled.setCursor(48,3);
      oled.print(shows);
      // Actualize Battery values
      oled.setCursor(10,7);
      oled << F("Battery ") << voltage << F("V ") << percent << F("%  ");
      oled.setCursor(0,7);
      if (batteryCritical) {
        oled.print("C");
      }
      else if (batteryWarning) {
        oled.print("W");
      }
      else {
        oled.print(" ");
      }
    }
    initStats = false;
  } // endif showStats

  // -------------------------- Show Setup -------------------------
  // Init
  if (showSetup) {
    // Do Init of Stats Page
    if (initSetup) {
      oled.clear();
      oled.print(F("<Setup>"));
      oled.setCursor(0,7);
      oled << F("Sleeptime: ") << SLEEPTIME << F("ms");
      setupElement[menuValueYear].value       = act_year;
      setupElement[menuValueMonth].value      = act_month;
      setupElement[menuValueDay].value        = act_day;
      setupElement[menuValueHour].value       = act_hr;
      setupElement[menuValueMinute].value     = act_min;
      // Clear the setupElements changed state
      for (int i=0; i < numberOfSetupElements; i++) {
        setupElement[i].changed = false;
      }
    } // endif initSetup

    // Show
    // Edit Mode On/Off
    if ( b1pos && getMenuEditMode(menuIndex) && !initSetup ) menuEditMode = !menuEditMode;

    // Show Active Menu Entry Inverted
    if ( b1pos && menuEditMode ) {
      oled.setInvertMode(true);
      oled.setCursor(menuTextPos,menuEditLine);
      oled.print(setupElement[menuIndex].name);
      oled.setInvertMode(false);
    }
    // Show Active Entry Not Inverted
    if ( b1pos && !menuEditMode ) {
      oled.setCursor(menuTextPos,menuEditLine);
      oled.print(setupElement[menuIndex].name);
    }
     // MenuIndex +/- by pressing Left Side Buttons shortly or hold for a longer time
    if (!menuEditMode) {
      if (b2pos || (b2long && blink500_pos)) menuIndex--;
      if (b3pos || (b3long && blink500_pos)) menuIndex++;
    }

    // MenuIndex Bounds
    if (menuIndex > menuEnd) menuIndex = menuStart;
    if (menuIndex < menuStart) menuIndex = menuEnd;
    // MenuIndex changed ?? => Update Menu & Cursor
    menuIndex_changed = menuIndex != prev_menuIndex?true:false;
    prev_menuIndex = menuIndex;

    // SetupElement Value +/- by pressing Left Side Buttons shortly or hold for a longer time
    if ( menuEditMode && getMenuEditMode(menuIndex) ) {
      if (b2pos || (b2long && blink250_pos)) {
        setupElement[menuIndex].value++;
        setupElement[menuIndex].changed = true;
      }
      if (b3pos || (b3long && blink250_pos)) {
        setupElement[menuIndex].value--;
        setupElement[menuIndex].changed = true;
      }

      // Actual setupElement[x].value changed ? => Update only the Value
      setupElementValue_changed = setupElement[menuIndex].value != prev_setupElementValue?true:false;
      prev_setupElementValue = setupElement[menuIndex].value;

      if ( setupElementValue_changed && menuEditMode ) {
        oled.setCursor(menuValuePos,menuEditLine);
        oled.clearToEOL();
        oled.setCursor(menuValuePos,menuEditLine);
        oled.print(setupElement[menuIndex].value);
        //sprintf(datebuffer, "%02u", setupElement[menuIndex].value);
        //oled.print(datebuffer);
      }
    }

    // V2
    if ((menuIndex_changed  && !menuEditMode) || initSetup) {
      // Show ">"
      oled.setCursor(0,menuEditLine);
      oled.print(">");
       for (int i=-2; i<=2; i++) {
        oled.setCursor(menuTextPos, menuEditLine + i);
        oled.clearToEOL();
        if ((menuIndex + i >= menuStart) && (menuIndex + i <= menuEnd)) {
        //if ((menuIndex + i > 0) && (menuIndex + i <= 11)) {
          oled.print(setupElement[menuIndex + i].name);
          if (getMenuEditMode(menuIndex + i)) {
            oled.setCursor(menuValuePos, menuEditLine + i);
            oled.print(setupElement[menuIndex + i].value);
            //sprintf(datebuffer, "%02u", setupElement[menuIndex + i].value);
            //oled.print(datebuffer);
          }  // endif getMenuEditMode
        }  // endif menuIndex
      }  // endfor
    }  // endif menuIndex_changed

    // Set Oled Contrast
    if (setupElementValue_changed && (menuIndex == menuValueContrast)) {
      oled.setContrast(setupElement[menuValueContrast].value);
    }

    // Little debugging and test "streaming"
    // oled.setCursor(0,7);
    // oled << "isAlarmDay: " << isAlarmDay;

    initSetup = false;
  } // endif showSetup

  // ------------------- Show Sensors --------------------
  if (showSensors) {
    // Init
    if(!mag.isCalibrated()) {
      // And we're not currently calibrating
      if(!mag.isCalibrating()) {
        oled.clear();
        oled.println("MAG Calibration Mode");
        oled.println("Rotate watchX 360°");
        mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
      }
      else {
        mag.calibrate();
      }
    }
    else {
      if (initSensors) {
        oled.clear();
        oled.print(F("<Sensors>"));
        oled.setCursor(6,2);
        oled.print("MAG: ");
        oled.setCursor(6,4);
        oled.print("AX:    AY:    AZ:");
        oled.setCursor(6,7);
        oled.print("T:");
        oled.setCursor(66,7);
        oled.print("A:");

      } // endif initSensors

      // Show Sensor Values each Seconds
      if (initSensors || secChanged) {
        // Show Angles
        oled.setCursor(0,5);
        oled.clearToEOL();
        oled.setCursor(6,5);
        oled.print(arx);
        oled.setCursor(48,5);
        oled.print(ary);
        oled.setCursor(90,5);
        oled.print(arz);

        // MAG3110
        heading = mag.readHeading();
        if (heading < 0) heading *= -1; //Negate the heading
        else if (heading > 0) heading = 360 - heading;
        oled.setCursor(36,2);
        oled << heading << " deg ";

        // BMP280
        oled.setCursor(24,7);
        oled.print(bme.readTemperature());
        oled.setCursor(84,7);
        oled.print(bme.readAltitude(1013.25));
      }
      initSensors = false;
    } // endif showSensors
  }

}  // endloop
