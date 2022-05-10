#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*================================================================================*/
/*BAROMETRIC PRESSURE SENSOR VARIABLES*/
#define CAL_TIMES 100 /*number of times to read for calibration*/
Adafruit_BMP3XX bmp;
double initial_pressure = 0;  /*blank value to store initial */
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
/*EEPROM INITIALISATION*/
/*eeprom can hold any value from 0 to 255*/
unsigned int address = 0; /*keep track of what EEPROM address we are using*/
double maxalt = 0.0; /*variable to store maximum altitude */

/*================================================================================*/
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Project Phobos flight computer: all sensors test"); Serial.println("");

/*================================================================================*/
/*Check BNO055 and BMP388 for connectivity, and initialise GPS*/
  if (!bno.begin())
  {
    Serial.print("No BNO055 data detected, check wiring!");
    while (1);
  }
  if (! bmp.begin_I2C())
  {  
    Serial.println("No BMP388 data detected, check wiring!");
    while (1);
  }
  ss.begin(GPSBaud);
  bno.set16GRange();
/*================================================================================*/
/*ALTIMETER SETUP*/
  // Set up oversampling and filter initialization  
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

/*================================================================================*/
  /*Read max altitude from EEPROM*/
  EEPROM.get(address,maxalt);
  Serial.print("maximum altitude = ");
  Serial.println(maxalt,3);
  
  /*sets the pressure when board turned on as zero altitude reference*/
  Serial.println("calibrating sensor...\n\n");
  /*ignore first few readings as they are WRONG*/
  for (int n=1; n<=500;n++)
  {
    bmp.readPressure();
    delay(1);
  }
  /*now calibrate the average pressure for the first CAL_TIMESx1ms of operation*/
  for (int s = 1; s <= CAL_TIMES; s++)
  {
    initial_pressure += bmp.readPressure();
    delay(1);
  }
  initial_pressure_HPa = initial_pressure/ CAL_TIMES / 100;
}

void loop() {
/*================================================================================*/
/*IMU data collecting and displaying*/
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  
/*================================================================================*/
/*GPS data collecting and displaying*/
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
/*================================================================================*/
/*Barometer data collecting and displating, plus writing to EEPROM*/
  /*read altitude and print to serial*/
  double newalt = bmp.readAltitude(initial_pressure_HPa);
  Serial.print("Approx . Altitude = ");
  Serial.print(newalt);
  Serial.println(" m");

  /*checks if max altitude has been exceeded, if so it sets new record and saves it to EEPROM address*/
  if(newalt>maxalt)
  {
    maxalt = newalt;
    EEPROM.put(address,maxalt);
  }
  delay(100);
}

/*================================================================================*/
/*Various functions for displaying data, likely won't be needed in the long run*/
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
