// controller.h
// drone estimator and controller
#include <math.h>
#ifndef CONTROLLER_H
#define CONTROLLER_H

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

typedef struct
{
  // let state x =[p,dP]
  // dx = [dp,d2p]
  float d_roll;
  float d_pitch;
  float d_yaw;

  float d2_roll;
  float d2_pitch;
  float d2_yaw;

} d_state;

typedef struct
{

  float throttle0;
  float throttle1;
  float throttle2;
  float throttle3;

} control;

typedef struct
  {
    //only PD, no I term yet
    state x;
    d_state dx;

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