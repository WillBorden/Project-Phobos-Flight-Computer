# Project-Phobos-Flight-Computer
Repository for software and documentation for the SRAD flight computer to be flows on the Project Phobos rocket at the 2022 Spaceport America Cup. We are currently running version 1.2 of the flight software.

The computer has the following capabilities and peripherals:
- Able to detect launch and landing, as well as apogee and main chute deployment altitude
- Can deploy up to two parachutes through the on-board pyro channels
- Can read and store data for an up to ? minute flight
- Stores data on an SD card and in non-volatile FRAM memory for redundancy
- Has a BNO055, BMP388, and NEO6M for acceleration, altitude, and GPS data respectively
- Has a buzzer and an RGB LED for status indication
- Can read LiPo battery voltage and check continuity across e-matches

# Necessary libraries/software to program the board
The following pieces of software and libraries are required to compile and write code to the computer:
- Arduino IDE - download from https://www.arduino.cc/en/software
- Teensyduino extention - download from https://www.pjrc.com/teensy/td_download.html
- NeoGPS library - https://github.com/SlashDevin/NeoGPS
- Adafruit Sensor library - https://github.com/adafruit/Adafruit_Sensor
- Adafruit BMP388 library - https://github.com/adafruit/Adafruit_BMP3XX
- Adafruit FRAM library - https://github.com/adafruit/Adafruit_FRAM_SPI

The Adafruit BNO055 library has been modified for this application, the modified version of the library can be downloaded from this repository, in a folder named Adafruit_BNO055-master
