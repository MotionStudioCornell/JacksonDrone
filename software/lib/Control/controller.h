// controller.h
// drone estimator and controller
#include <math.h>
#ifndef CONTROLLER_H
#define CONTROLLER_H

#define VECTOR_SIZE 3 // Size of the vectors for angular velocity and acceleration (xyz)
#define PI 3.14159265358979323846

typedef struct{
  //right head rule with imu label
  float roll;
  float pitch;
  float yaw;

  //complementary filter coeff [0,1]
  float com_alpha;
  //sampling time in sec
  float dT;

  float Kp;
  float Ki;
  float Kd;

} controller;

void init_controller(controller* my_controller, float alpha, float dT, float Kp, float Ki, float Kd);

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE]);

#endif // CONTROLLER_H