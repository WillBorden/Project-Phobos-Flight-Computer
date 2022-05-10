//A quick and dirty sketch that roughly simulates a rocket flight, (including simulated sensor noise) can be used to test filters and data storage for the Project Phobos flight computer
//written by Will Borden, 22/03/2022, wborden@student.unimelb.edu.au
//this sketch makes many assumptions such as no air resistance, perfectly vertical flight, constant thrust, etc. It is only to be used for generating rough test values to validate that things are working.
//if you want to use values without noise, just use variables a, v, and s, instead of a_n, v_n, and s_n
#include <Gaussian.h>

double a = 0.0; //acceleration
double v = 0.0; //velocity
double s = 0.0; //vertical displacement

double a_n = 0.0; //acceleration with noise
double v_n = 0.0; //velocity with noise
double s_n = 0.0; //vertical displacement with noise


double t = 0.05; //time step`

//gaussian objects for sensor noise
Gaussian a_gaussian = Gaussian(0,0.0346);
Gaussian v_gaussian = Gaussian(0,2);
Gaussian s_gaussian = Gaussian(0,5.985);



void setup() {
  Serial.begin(115200); //initialise serial, wait for monitor to be opened
  while(!Serial);

  for(int i = 0; i <= 56; i++){ //increasing i gives longer burn of motor
    //powered flight, a is the net acceleration upwards
    a = 50;
    v = v + a*t;
    s = s + v*t;

    a_n = a + a_gaussian.random();
    v_n = v + v_gaussian.random();
    s_n = s + s_gaussian.random();
    Serial.print(a_n);
    Serial.print('\t');
    Serial.print(v_n);
    Serial.print('\t');
    Serial.println(s_n);
  }

  while(s >= 0){
    //free-fall
    a = -9.8;
    v += a*t;
    s += v*t;

    a_n = a + a_gaussian.random();
    v_n = v + v_gaussian.random();
    s_n = s + s_gaussian.random();
    Serial.print(a_n);
    Serial.print('\t');
    Serial.print(v_n);
    Serial.print('\t');
    Serial.println(s_n);
  }
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
