//Test code for a Kalman filter to be implemented  on the flight computer for Project Phobos
//Written by Will Borden, 21/03/2022, wborden@student.unimelb.edu.au
//Currently working, however all noise values are arbitrarily chosen. Further tweaking will be required to get this working properly
//The state of the rocket in this filter is approximated by the equations s=ut+1/2at^2, v=at, and a=-g. As air resistance ~ 0 as v approaches 0, these should be suitable for apogee detection.
//This code gets state values from the same code in Dummy_a_and_s.ino

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Gaussian.h>
#include "data.h"

//arrays with real data

//initialise BasicLinearAlgebra library
using namespace BLA;

//simulation variables
float a = 0.0; //acceleration
float v = 0.0; //velocity
float s = 0.0; //vertical displacement

float a_n = 0.0; //acceleration with noise
float v_n = 0.0; //velocity with noise
float s_n = 0.0; //vertical displacement with noise

//variables for sensor noise
Gaussian a_gaussian = Gaussian(0,0.0346); //if noise is not wanted, set both numbers inside the brackets to 0
Gaussian v_gaussian = Gaussian(0,2);
Gaussian s_gaussian = Gaussian(0,5.985);//5.985

//define necessary matrices and variables
BLA::Matrix<3,1> x; //State matrix
BLA::Matrix<2,1> m; //measured altitude and acceleration
  
BLA::Matrix<3,2> K; //Kalman Gains
BLA::Matrix<2,3> H; //maps state variables to sensor data
BLA::Matrix<2,2> R; //measurement noise covariance
BLA::Matrix<3,3> Q; //process noise covariance matrix
BLA::Matrix<3,3> I; //3x3 identity matrix
BLA::Matrix<3,3> P; //error covariance matrix

BLA::Matrix<2,2> C;

float T = 0.03;

BLA::Matrix<3,3> A; //maps previous state to next state
  
void setup() {

  Serial.begin(115200);
  while(!Serial); //wait for serial connection

  x = {0.0,0.0,0.0};
  H = {1.0,0,0,0,0,1.0};
  R = {0.186217677,0,0,0.000107007}; //dummy values for the moment
  Q = {0,0,0,0,0,0,0,0,1.0};
  I = {1.0,0,0,0,1.0,0,0,0,1.0};
  P = I; //initial guess for P
  A = {1,T,(1.0/2.0)*T*T,0,1,T,0,0,1};

  //the following 3 equations recursively calculate K and P, this can be done ahead of time to reduce strain on processor during flight:
  for(int i = 0; i <= 20; i++){
    C = H*P*~H + R;
    K = P*~H*Invert(C);
    P = (I - K*H)*P;
    P = A*P*~A + Q;
  }

  for(int i=0; i <= 2731; i++){

    Serial.print(position_data(i));
    Serial.print('\t');  
    Serial.print(acceleration_data(i));
    Serial.print('\t');  

    //give real values to filter
    T = dt(i);
    A = {1,T,(1.0/2.0)*T*T,0,1,T,0,0,1};
    m = {position_data(i),acceleration_data(i)};
    //actual filter implementation
    x = A*x;
    x = x + K*(m - H*x);
    Serial.print(x(0));
    Serial.print('\t');
    Serial.print(x(1));
    Serial.print('\t');
    Serial.println(x(2));
  }
/*
  for(int i = 0; i <= 75; i++){ //increasing i gives longer burn of motor
    //powered flight, a is the net acceleration upwards
    a = 25;
    v = v + a*T;
    s = s + v*T;

    a_n = a + a_gaussian.random();
    v_n = v + v_gaussian.random();
    s_n = s + s_gaussian.random();
    Serial.print(a_n);
    Serial.print('\t');
    Serial.print(v_n);
    Serial.print('\t');
    Serial.print(s_n);
    Serial.print('\t');

    //give simulated values to filter
    m = {s_n,a_n};
    //actual filter implementation
    x = A*x;
    x = x + K*(m - H*x);
    Serial.print(x(0));
    Serial.print('\t');
    Serial.print(x(1));
    Serial.print('\t');
    Serial.println(x(2));
  }

  while(s >= 0){
    //free-fall       
    a = -9.8;
    v = v + a*T;
    s = s + v*T;

    a_n = a + a_gaussian.random();
    v_n = v + v_gaussian.random();
    s_n = s + s_gaussian.random();
    Serial.print(a_n);
    Serial.print('\t');
    Serial.print(v_n);
    Serial.print('\t');
    Serial.print(s_n);
    Serial.print('\t');
    
    //give simulated values to filter
    m = {s_n,a_n};
    //actual filter implementation
    x = A*x;
    x = x + K*(m - H*x);
    Serial.print(x(0));
    Serial.print('\t');
    Serial.print(x(1));
    Serial.print('\t');
    Serial.println(x(2));

  }  */
  /*
   Serial << "K: " << K << '\n';
   Serial << "H: " << H << '\n';
   Serial << "P: " << P << '\n';
  */

}

void loop() {
  // put your main code here, to run repeatedly:
  
}
