//Third version of the Project Phobos flight computer, to be ran for the first time on 15/05/22
//written by Will Borden, 26/03/22, wborden@student.unimelb.edu.au
//This code reads values from sensors and records them to external FRAM memory as well as the EEPROM then writes to SD card apon touchdown.
//It uses a Kalman filter to detect liftoff, landing, and apogee, and deploys the main and drogue parachutes

#include <NMEAGPS.h>
#include <GPSport.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_FRAM_SPI.h>
#include <utility/imumaths.h>

/*Rocket States:
 STATE: | DESCRIPTION:                         | COLOUR: | BUZZER:
 ----------------------------------------------------------------------------------------------------------------------
    0   | powered on; initialising sensors     | RED     | 2 tones on power up
    1   | ready; no guaranteed GPS lock        | GREEN   | 4 tones when ready, one beep every 2 seconds once GPS locked
    2   | launch detected; data being stored   | BLUE    | N/A
    3   | EEPROM/Array full; stop storing data | PURPLE  | N/A
    4   | Landing detected; write to SD card   | YELLOW  | Buzzer on
    5   | Write successful                     | WHITE   | N/A
*/

int state = 0; //rocket state, see above table

int write_delay = 20; //number of milliseconds we want to wait between storing data, limits how fast we fill up memory

int launch_alt = 0; //min altitude that counts as a launch
int landing_alt = 0; //max altitude that counts as a landing

float v_bat; //battery voltage
int voltage_read = 0;
int batteryPin = 38;

/*================================================================================*/
/*LED AND BUZZER PINS*/
int redPin= 2;
int greenPin = 4;
int bluePin = 5;

int buzzerPin = 8;
bool buzzer_on = false;

/*================================================================================*/
/*BAROMETRIC PRESSURE SENSOR VARIABLES*/
#define CAL_TIMES 100 //number of times to read for barometer calibration
Adafruit_BMP3XX bmp;
double initial_pressure = 0;  //blank value to store initial pressure reading
double initial_pressure_HPa = 0;

/*================================================================================*/
/*IMU VARIABLES*/
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //delay between fresh samples
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/*================================================================================*/
/*GPS VARIABLES*/
static const int RXPin = 0, TXPin = 1; 
NMEAGPS  gps; // this parses the GPS characters
gps_fix  fix; // holds GPS values
SoftwareSerial ss(RXPin, TXPin); //serial connection to the GPS device

float latitude = 0; 
float longitude = 0;
float gps_alt = 0;
String date_string;

/*================================================================================*/
/*FRAM AND SD CARD VARIABLES*/
unsigned int address = 0;
uint8_t FRAM_CS = 10;
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS); //initialise FRAM

const int chipSelect = BUILTIN_SDCARD; // SD card has an internal chipselect for the Teensy
String file_name;

/*================================================================================*/
/*TIMING VARIABLES*/
float dt, currentmillis, previousmillis;
int buzzer_previous = 0;

/*================================================================================*/

void setup() {
  //startup tone
  tone(buzzerPin,659);
  delay(250);
  noTone(buzzerPin);
  tone(buzzerPin,880,250);
  
  //Serial.begin(115200);
  //Serial.println("Project Phobos flight software V1.2"); Serial.println("");
/*================================================================================*/
/*Pinmodes*/
  pinMode(redPin,OUTPUT);
  pinMode(bluePin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  pinMode(batteryPin,INPUT);
  
/*================================================================================*/
/*sensor and SD initialisation*/
  setColour(255,0,0); //red
  gpsPort.begin(9600); //initialise GPS, NEO-6M must be set to 9600 baud
  //check that other modules initialise
  if(bno.begin() && bmp.begin_I2C() && SD.begin(chipSelect) && fram.begin()){
    bno.set16GRange(); //configure IMU to +/-16G range with custom function
    /*Barometer calibration*/
    //sets the pressure when board turned on as zero altitude reference
    //ignore first few readings as they are WRONG
    for(int i = 1; i <= 500; i++){
      bmp.readPressure();
      delay(1);
    }
    //now calibrate the average pressure for the first CAL_TIMESx1ms of operation
    for(int s = 1; s <= CAL_TIMES; s++){
      initial_pressure += bmp.readPressure();
      delay(1);
    }
    initial_pressure_HPa = initial_pressure/ CAL_TIMES / 100;
    
    state = 1; //rocket is ready for launch
    
    //buzzer tone confirming successful initialisation
    tone(buzzerPin,880);
    delay(150);
    noTone(buzzerPin);
    tone(buzzerPin,1109);
    delay(150);
    noTone(buzzerPin);
    tone(buzzerPin,880);
    delay(150);
    noTone(buzzerPin);
    tone(buzzerPin,1319);
    delay(150);
    noTone(buzzerPin);
    delay(500);
  }

}
/*================================================================================*/
  

void loop() {
/*================================================================================*/
/*timing*/
  currentmillis = millis(); 
  previousmillis = currentmillis;
  dt = (currentmillis - previousmillis) / 1000; //will be used for Kalman filter
  
/*================================================================================*/
/*IMU data collecting*/
  //most of this is currently unused, this probably increases the loop time so we might want to get rid of what we don't need
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //3x1 matrix that holds acceleration data
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float acceleration_vertical = acceleration.x();
  //boards temparature
  //int8_t boardTemp = bno.getTemp();
  //calibration data
  //uint8_t system, gyro, accel, mag = 0;
  //bno.getCalibration(&system, &gyro, &accel, &mag);
/*================================================================================*/
/*GPS data collecting*/
  while (gps.available(gpsPort)) {
    fix = gps.read();

    latitude = fix.latitude();
    longitude = fix.longitude();
    gps_alt = fix.altitude();
    //date_string = String(fix.dateTime.date) + "/" + String(fix.dateTime.month) + "/" + String(fix.dateTime.year) + "_" + String(fix.dateTime.hours) + ":" + String(fix.dateTime.minutes) + ":" + String(fix.dateTime.seconds);
    
  }
/*================================================================================*/
/*Barometer data collecting*/
  float alt = bmp.readAltitude(initial_pressure_HPa);
/*================================================================================*/
/*Liftoff and landing detection*/
  if(state == 1 && alt > launch_alt){
    //liftoff
    noTone(buzzerPin); //turn of buzzer in case it's on when launch is detected
    delay(100);
    state = 2;
  }
  if((state == 2 || state == 3) && alt < landing_alt){
    //landed
    state = 4;
    setColour(255,100,0); //yellow
    tone(buzzerPin,1109);
    //after landing is detected record an additional 300 samples to collect all data
    for(int i = 0; i <= 300; i++){
      imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      acceleration_vertical = acceleration.x();
      while (gps.available(gpsPort)) {
      fix = gps.read();
      latitude = fix.latitude();
      longitude = fix.longitude();
      gps_alt = fix.altitude();
      }
      alt = bmp.readAltitude(initial_pressure_HPa);
      currentmillis = millis();
      
      FRAM_write(address*4,currentmillis/1000);
      address++;
      FRAM_write(address*4,latitude); 
      address++;
      FRAM_write(address*4,longitude);
      address++;
      FRAM_write(address*4,acceleration_vertical);
      address++;
      FRAM_write(address*4,alt);
      address++;
      FRAM_write(address*4,gps_alt);
      address++;
      delay(write_delay);
    }
    //write array to SD card, in CSV format so we can open it into Excel
    Serial.println(file_name);
    File dataFile = SD.open("launch_data.csv", FILE_WRITE);
    dataFile.println("time,latitude,longitude,vertical acc,alt,gps alt");
    String dataString = "";
    for(int i = 0; i < 262144; i=i+24){ 
      dataString = String(FRAM_read(i),8) + "," + String(FRAM_read(i+4),8) + "," + String(FRAM_read(i+8),8) + "," + String(FRAM_read(i+12),8) + "," + String(FRAM_read(i+16),8) + "," + String(FRAM_read(i+20),8);
      dataFile.println(dataString);
      Serial.println(dataString);
      //Serial.println(dataString);
    }
    dataFile.close();
    Serial.println("Data written to SD card");
    state = 5; //successful write!
  }
/*================================================================================*/
/*begin writing to FRAM, check if full*/
  if(state == 2){
      //write to FRAM
      FRAM_write(address*4,currentmillis/1000); //advance FRAM address by 4 each time, as floats are written over 4 bytes
      address++;
      FRAM_write(address*4,latitude); 
      address++;
      FRAM_write(address*4,longitude);
      address++;
      FRAM_write(address*4,acceleration_vertical);
      address++;
      FRAM_write(address*4,alt);
      address++;
      FRAM_write(address*4,gps_alt);
      address++;
    delay(write_delay); //stops memory from filling up too fast, in future external memory will be implemented so we can sample more quickly
  }
  if(address == 524288){ //check if FRAM is full
    //stop writing to FRAM
    state = 3; 
  }
/*================================================================================*/
/*Change LED colour*/
  if(state == 0){
    setColour(255,0,0); //red
  }
  if(state == 1){
    setColour(0,255,0); //green
    //check to see if we have GPS connection
    if(fix.valid.location){
    //if we do have GPS fix, set off buzzer every 2 seconds
    if((currentmillis - buzzer_previous) >= 2000){
      buzzer_previous = currentmillis;
      buzzer_on = true;
    }
    if(buzzer_on = true && (currentmillis - buzzer_previous) <= 250){
      tone(buzzerPin,880);
    }
    else{
      noTone(buzzerPin);
    }
    }
  }
  if(state == 2){
    setColour(0,0,255); //blue
  }
  if(state == 3){
    setColour(255,0,255); // purple
  }
  if(state == 4){
    setColour(255,100,0); //yellow
  }
  if(state == 5){
    setColour(255,255,255); //white
  }
/*================================================================================*/
/*Read battery voltage*/
  voltage_read = analogRead(batteryPin);
  v_bat = map(voltage_read,0,1009,0,5000) / 1000.0;
/*================================================================================*/
/*Serial print important values*/
  //Serial.print("state: "); Serial.println(state);
  //Serial.print("lat: "); Serial.println(longitude, 6);
  //Serial.print("lng: "); Serial.println(latitude, 6);
  //Serial.print("date: "); Serial.println(file_name);
  //Serial.print("acc: "); Serial.println(acceleration_vertical);
  //Serial.print("alt: "); Serial.println(alt);
  //Serial.print("millis: "); Serial.println(millis());
  //Serial.print("battery voltage: "); Serial.println(v_bat,5);
}

//functions to write floats to FRAM, NOTE: variables written to FRAM must be floats, not doubles
template <class T> int FRAM_write(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++){
        fram.writeEnable(true);
        fram.write8(ee++, *p++);
        fram.writeEnable(false);
    }
    return i;
}

float FRAM_read(int ee)
{
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        *p++ = fram.read8(ee++);
    return value;
}
//write colour to RGB LED
void setColour(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}
