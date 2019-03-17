
SleepyClock 0.1.7/0.1.6
based on TapClock 0.8.x
License: MIT
Get Support: https://community.watchx.io/
Using Atom Editor and PlatformIO (PIO)

Very Urgent !!
You need an calibrated MPU(IMU), see "Calibration".

Files & Folder
.\src                         Main Source file "main.cpp"
.\lib                         Manually added libs like my own "Edge" lib
.\include                     Manually added .h files (#include..) like the font or the calibration data files

Libraries:
You can add libraries to your PIO Project by adding them with the statement "lib_deps" to the file platformio.ini.
PIO will then download the libs automatically.
Or you download the libs manually to the "global" storage with "platformio lib -g install lib-name1, lib-name2....".

Buttons
   /---------------\
[1]|               |[2]
   |    watchX     |
   |               |[3]
   \---------------/

How it works:
The Loop
  watchX's MCU and MPU are permanently send to sleep.
  The MCU is woken up by the internal WatchDog timer after SLEEPTIME (see define's).
  If watchX is up, the MPU's Sleep Mode is disabled and watchX's Position is checked.
  If the Position matches the "Read Position" (see Variables), the Display is powered on and shows the clock for a CLOCKTIME (see define's) time.
  If the Position doesn't match, the MPU Sleep Mode is enabled and the MCU goes back to sleep as well...

USB Power prevents the System from sleeping

HowTo Use:
-Bring watchX in an readable Position (x: 25-85°, y: +/-10°, z doesn't matter) to get the Clock screen shown.
-If the Clock is up, the Upper Left Button (1) opens the Setup Menu to adjust values.
-If the Clock is up, the Upper Right Button (2) opens the Stats Menu.
 -In "Stats" the Lower Right Button (3) Resets Uptime and Wakes
 -In "Stats" you see:
  -Actual Time
  -Uptime
  -Shows: show the "Show Clock" counter
  -Angles X, Y, Z
-With the Upper Left Button (1) you go back to the "Clock" Screen.

USB Connection:
If the system is woken up and clock is shown, the command "USBDevice.attach();" is processed to get the lost USB connection back.
Means you should/could transfer your sketch without Resetting watchX for Sketch transfer.
See here https://github.com/adafruit/Adafruit_SleepyDog/blob/master/README.md for Details.

Charging:
Battery charging is shown with an glowing Right Led.
If watchX is connected to USB Power the System will never go to sleep if it’s "On".
You can wait until the display is powered off and then connect USB Power for charging without “display on”.

Battery Warning/Alarm:
-If the Battery goes under BATTERY_WARNING the Battery Icon starts to blink.
-If the Battery goes over BATTERY_OK the Battery Icon stops blinking

Calibration:
Calibrate your MPU as each MCU is different.
Get the calibration sketch from here: https://github.com/venice1200/TapClock/tree/master/CalibratingSketch

You need to copy the calibration values out of the serial window (9600 baud) of the calibration sketch.
Open the file "MPU_CalVal_watchX.h" in "include" and replace the below values with yours
#define mpu_XAccelOffset  106
#define mpu_YAccelOffset -397
#define mpu_ZAccelOffset  1255
#define mpu_XGyroOffset   68
#define mpu_YGyroOffset  -9
#define mpu_ZGyroOffset   29

Look here https://i.imgur.com/FOmkXfg.png to see which values are needed
You have to choose one of the calibration values for each section.

After you have replaced the calibration values with yours, compiled and uploaded the sketch
the Stats Screen should show nearly 0 at each Angle if the watchX lies on a flat and horizontal surface.
See https://i.imgur.com/mNhEpCd.jpg

See also the calibration sketch header as there are some more infos
like heating up the MPU for 10 Minutes before you run the calibrating sketch
and use a flat and horizontal surface for calibration.

SLEEPTIME:
An longer SLEEPTIME keeps watchX longer on battery.
With an shorter SLEEPTIME you get a better wakeup reaction.
Values of around 60/120/250  and 500ms are good in case of reaction time and battery life.
With  60ms SLEEPTIME the System runs for around xxx hrs
With 120ms SLEEPTIME the System runs for around 30 hrs (10% Battery)
With 250ms SLEEPTIME the System runs for around 41 hrs
With 500ms SLEEPTIME the System runs for around xx hrs, Display "Power On" can take up to two seconds.

Libraries:
-watchX libs by ArgeX                   see https://watchx.io/download.php
-I2CDEVLIB & MPU6050                    see https://github.com/jrowberg/i2cdevlib
-DS3232RTC RTC Library                  see https://github.com/JChristensen/DS3232RTC
-Time (needed by RTC lib)               see https://github.com/PaulStoffregen/Time
-SSD1306 Text Library                   see https://github.com/greiman/SSD1306Ascii
-Adafruit Sleepy Dog                    see https://github.com/adafruit/Adafruit_SleepyDog
-Streaming by Mikal Hart                see https://github.com/geneReeves/ArduinoStreaming fork of Mikal Hart's streaming5.zip
-Edge by me                             see https://github.com/venice1200/Edge

PlatformIO library list, see platformio.ini
lib_deps =
  I2Cdevlib-Core
  I2Cdevlib-MPU6050
  DS3232RTC
  Time
  SSD1306Ascii
  Adafruit SleepyDog Library
  Streaming
  SparkFun MAG3110 Magnetometer Breakout Arduino Lib

Tips & Tricks:
-Angle calculation                      see https://electronics.stackexchange.com/questions/142037/calculating-angles-from-mpu6050
-Set Time at "Setup"                    see http://www.l8ter.com/?p=417
-Nick Gammon Microprocessors Infos      see http://www.gammon.com.au/power and http://www.gammon.com.au/interrupts

Credits:
A watchX Sketch based on watchX Hardware and:
watchX libs provided by ArgeX
OLED Library SSD1306Ascii by Greiman
i2cdevlib/mpu6050 by jrowberg
DS3232RTC Library by JChristensen
Time maintained by Paul Stoffregen
Streaming Lib by Mikal Hart
Adafruit
Arduino Team
The watchX Reddit Community

===================================================

ToDo:
-Build System around (in Progress)
-Setup System (via Bluetooth?)
-RTC Alarm
-Battery Alarm
-Testing with original watchX MPU library
