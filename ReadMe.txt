
  SleepyClock 0.1.5
  based on TapClock 0.8.x
  License: MIT
  Get Support: https://community.watchx.io/

  Very Urgent !!
  You need an calibrated MPU, see setup().
  Get the Calibration Sketch from here: https://github.com/venice1200/TapClock/blob/master/CalibratingSketch/WatchX_IMU_Zero_0x69.ino
  See ReadMe of TapClock https://github.com/venice1200/TapClock/blob/master/TapClock/PreviousVersions/TapClock_v0.8.4/TapClock_v0.8.4_ReadMe.txt and search for "calibration".
  Add you Calibration Values to the MPU_CalVal_watchX.h file and make sure this file is "included".

  Used Libaries:
   -watchX libs by ArgeX                   see http://watchx.io/downloads_en.html
   -I2CDEVLIB & MPU6050                    see https://github.com/jrowberg/i2cdevlib
   -DS3232RTC RTC Library                  see https://github.com/JChristensen/DS3232RTC
   -Time (needed by RTC lib)               see https://github.com/PaulStoffregen/Time
   -SSD1306 Text Library                   see https://github.com/greiman/SSD1306Ascii
   -Adafruit Sleepy Dog                    see https://github.com/adafruit/Adafruit_SleepyDog
   -Streaming by Mikal Hart                see https://github.com/geneReeves/ArduinoStreaming fork of Mikal Hart's streaming5.zip
   -Edge by me                             see https://github.com/venice1200/Edge

   -PlatformIO libs:
   lib_deps = I2Cdevlib-Core, I2Cdevlib-MPU6050, DS3232RTC, Time, SSD1306Ascii, Adafruit SleepyDog Library, Streaming, SparkFun MAG3110 Magnetometer Breakout Arduino Lib

   Buttons:
      /---------------\
   [1]|               |[2]
      |    watchX     |
      |               |[3]
      \---------------/

  How it works:
  while (true) {
    watchX's MCU is send to sleep and woken up by the WatchDog timer after SLEEPTIME (see define's).
    If watchX MCU is up the MPU Sleep Mode is disabled and watchX's Position is checked.
    If the Position matches the "Read Position" (see Variables) the Display is powered on and shows the clock for a CLOCKTIME (see define's) time.
    If the Position doesn't match the MPU Sleep Mode is enabled and the MCU goes back to sleep as well...
  }

  An longer SLEEPTIME keeps watchX longer on battery.
  With an shorter SLEEPTIME you get a better wakeup reaction.
  Values of around 60/120/250  and 500ms are good in case of reaction time and battery life.
  With  60ms SLEEPTIME the System runs for around xxx hrs
  With 120ms SLEEPTIME the System runs for around 30 hrs (10% Battery)
  With 250ms SLEEPTIME the System runs for around 41 hrs
  With 500ms SLEEPTIME the System runs for around xx hrs

  USB Power prevents the System from sleeping

  USB Connection:
  If the system is woken up, the command "USBDevice.attach();" is processed to get the lost USB connection back.
  Means you should/could transfer your sketch without Resetting watchX for Sketch transfer.
  See here https://github.com/adafruit/Adafruit_SleepyDog/blob/master/README.md for Details.

  HowTo Use:
  -Bring watchX in an readable postion (x: 25-85°, y: +/-10°, z doesn't matter) to see the Clock screen.
  -If the Clock is up, the Upper Left Button (1) opens the Setup Menu to adjust values.
  -If the Clock is up, the Upper Right Button (2) opens the Stats Menu.
   -In "Stats" the Lower Right Button (3) Resets Uptime and Wakes
   -In "Stats" you see:
    -Actual Time
    -Uptime
    -Shows: show the "Show Clock" counter
    -Angles X, Y, Z
  -With the Upper Left Button (1) you go back to the "Clock" Screen.

  Battery Warning/Alarm
  -If the Battery goes under BATTERY_WARNING the Battery Icon starts to blink.
  -If the Battery goes over BATTERY_OK the Battery Icon stops blinking

  Work in progress...

  Versions:
  v0.1.3
   -Move MPU Calibration values into .h file, no need to modify the sketch

  v0.1.5 (75%/38% Memory)
   -Added Setup Menu (actualy only) for adjusting Time

  ToDo:
   -Build System around (in Progress)
   -Setup System (via Bluetooth?)
   -RTC Alarm
   -Battery Alarm
   -Testing with original watchX MPU library

  Hints:
   -The MAG Object and Sleep Code uses 2% Sketch
