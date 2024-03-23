// controller.h
// drone estimator and controller
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <math.h>
#include <stdio.h>
#define VECTOR_SIZE 3 // Size of the vectors for angular velocity and acceleration (xyz)
#define PI 3.14159265358979323846

typedef struct
{
  // define p = [roll, pitch, yaw]^T
  // let state x =[p,dP]
  // right head rule with imu label
  float roll;
  float pitch;
  float yaw;

  float d_roll;
  float d_pitch;
  float d_yaw;

} state;

// throttle for each motor [0,100]
// Assume 
//          roll
//        t4   ^   t2
//             |
//  pitch <--- |
//        t3       t1

typedef struct
{

  float t1;
  float t2;
  float t3;
  float t4;

} control;

typedef struct
{
  // only PD, no I term yet
  state x;      // current state
  state target; // target state

  state prev_diff; // previous (target - x), use to calcuate d/dt diff 

  control u;

  // complementary filter coeff [0,1]
  float com_alpha;
  // sampling time in sec
  float dT;

  float Kp;
  float Ki;
  float Kd;

} controller;

void init_controller(controller *my_controller, float alpha, float dT, float Kp, float Ki, float Kd);

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE]);

#endif // CONTROLLER_H