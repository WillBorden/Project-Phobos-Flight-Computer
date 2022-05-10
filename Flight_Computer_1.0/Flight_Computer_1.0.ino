//First version of the Project Phobos flight computer, to be ran on the board for the first time on 27/03/22
// written by Will Borden, 26/03/22, wborden@student.unimelb.edu.au
//This code reads values from sensors and records them to an array, then writes to SD card apon touchdown
//Has rudimentary liftoff detection and landing detection, will be improved in future versions through filtering
//DOES NOT use the radio module
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int state = 0; //state 0 indicates on launchpad, state 1 indicates liftoff, state 2 indicates landing, state 3 indicates full array
int write_delay = 50; //number of milliseconds we want to wait between storing data to the array
int launch_alt = 10; //min altitude that counts as a launch

double data_array[16384];
const int chipSelect = BUILTIN_SDCARD; // SD card has an internal chipselect for the Teensy

/*================================================================================*/
/*BAROMETRIC PRESSURE SENSOR VARIABLES*/
#define CAL_TIMES 100 /*number of times to read for barometer calibration*/
Adafruit_BMP3XX bmp;
double initial_pressure = 0;  /*blank value to store initial pressure reading*/
double initial_pressure_HPa = 0;

/*================================================================================*/
/*IMU VARIABLES*/
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; /*delay between fresh samples*/
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/*================================================================================*/
/*GPS VARIABLES*/
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600; /*NEO-6M must be set to 9600 baud*/
TinyGPSPlus gps; /*The TinyGPSPlus object*/
SoftwareSerial ss(RXPin, TXPin); /*The serial connection to the GPS device*/

/*================================================================================*/
/*Array variables*/
unsigned int address = 0;

/*================================================================================*/

void setup() {
  //Serial.begin(115200);
  //Serial.println("Project Phobos flight software V1.0"); Serial.println("");
/*================================================================================*/
/*sensor and SD initialisation*/
  bno.begin();
  bmp.begin_I2C();
  ss.begin(GPSBaud);
  SD.begin(chipSelect);
/*================================================================================*/
  /*Barometer calibration*/
  /*sets the pressure when board turned on as zero altitude reference*/
  //Serial.println("calibrating sensor...\n\n");
  /*ignore first few readings as they are WRONG*/
  for(int i = 1; i <= 500; i++){
    bmp.readPressure();
    delay(1);
  }
  /*now calibrate the average pressure for the first CAL_TIMESx1ms of operation*/
  for(int s = 1; s <= CAL_TIMES; s++){
    initial_pressure += bmp.readPressure();
    delay(1);
  }
  initial_pressure_HPa = initial_pressure/ CAL_TIMES / 100;
}

void loop() {
/*================================================================================*/
/*IMU data collecting*/
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  //3x1 matrix that holds acceleration data
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double acceleration_vertical = acceleration.x();
  //boards temparature
  //int8_t boardTemp = bno.getTemp();
  //calibration data
  //uint8_t system, gyro, accel, mag = 0;
  //bno.getCalibration(&system, &gyro, &accel, &mag);
/*================================================================================*/
/*GPS data collecting*/
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  //double gps_alt = gps.altitude.meters();
  //double gps_time = gps.time.value();
/*================================================================================*/
/*Barometer data collecting*/
  double alt = bmp.readAltitude(initial_pressure_HPa);
/*================================================================================*/
/*Liftoff and landing detection*/
  if(state == 0 && alt > launch_alt){
    //liftoff
    delay(100);
    state = 1;
  }
  if((state == 1 || state == 3) && alt < launch_alt - 5){
    //landed
    delay (3000);
    state = 2;
    //write array to SD card
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    String dataString = "";
    for(int i = 0; i < 16384; i=i+4){
      dataString = String(data_array[i],8) + "  " + String(data_array[i+1],8) + "     " + String(data_array[i+2]) + "  " + String(data_array[i+3]);
      dataFile.println(dataString);
      //Serial.println(dataString);
    }
    dataFile.close();
    //Serial.println("Data written to SD card");
    while(1);
  }
/*================================================================================*/
/*begin writing to array, check if array is full*/
  if(state == 1){
    data_array[address] = latitude;
    address++;
    data_array[address] = longitude;
    address++;
    data_array[address] = acceleration_vertical;
    address++;
    data_array[address] = alt;
    address++;
    delay(write_delay);
  }
  if(address == 16383){ //check if array is full
    //stop writing to array
    state = 3;
  }
/*================================================================================*/
/*Serial print important values*/
Serial.print("state: "); Serial.println(state);
Serial.print("lat: "); Serial.println(latitude);
Serial.print("lng: "); Serial.println(longitude);
Serial.print("acc: "); Serial.println(acceleration_vertical);
Serial.print("alt: "); Serial.println(alt);
}
