/* 
 * This code is a minimal empty shell.
 * Fill it and modify it to match your own case.
 * 
 * Author:
 *  R.JL. Fétick
 * 
 * Revision:
 *  24 Aug 2019 - Creation
 */
#include "data.h"
#include <Kalman.h>
using namespace BLA;

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------

// Dimensions of the matrices
#define Nstate 3 // length of the state vector
#define Nobs 2   // length of the measurement vector

// measurement var (to be characterized from your sensors)
#define n_p 0.186217677 // noise on the 1st measurement component
#define n_a 0.000107007 // noise on the 2nd measurement component 

// model var (~1/inertia). Freedom you give to relieve your evolution equation
#define m_p 0.1
#define m_v 0.1
#define m_a 0.05
float DT;

KALMAN<Nstate,Nobs> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector

BLA::Matrix<Nstate> state; // true state vector

// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.


//-----------------------------------
/****           SETUP           ****/
//-----------------------------------

void setup() {

  Serial.begin(11520);
  
  // time evolution matrix (whatever... it will be updated inloop)
  K.F = {1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  // measurement matrix n the position (e.g. GPS) and acceleration (e.g. accelerometer)
  K.H = {1.0, 0.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_p,   0.0,
         0.0,  n_a};
  // model covariance matrix
  K.Q = {m_p*m_p, 0.0, 0.0,
         0.0, m_v*m_v, 0.0,
         0.0, 0.0, m_a*m_a};


  //calculate data
  for(int i = 0; i<=2731; i++){
    DT = dt(i);

    K.F = {1.0,  DT,  DT*DT/2,
           0.0,  1.0,      DT,
           0.0,  0.0,    1.0};

    obs(0) = position_data(i);
    obs(1) = acceleration_data(i);

    K.update(obs);

    Serial.print(obs(0));
    Serial.print('\t');
    Serial.print(obs(1));
    Serial.print('\t');
    Serial.print(K.x(0));
    Serial.print('\t');
    Serial.print(K.x(1));
    Serial.print('\t');
    Serial.print(K.x(2));
    Serial.println('\t');
    
    
  }
}
void loop() {
  
}
