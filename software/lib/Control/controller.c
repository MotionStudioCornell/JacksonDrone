// controller.c
#include "controller.h"

void init_controller(controller *my_controller, float alpha, float dT, float Kp, float Ki, float Kd)
{
  my_controller->com_alpha = alpha;
  my_controller->dT = dT;
  my_controller->Kp = Kp;
  my_controller->Ki = Ki;
  my_controller->Kd = Kd;

  //setting body frame the reference frame
  my_controller->roll = 0.0;
  my_controller->pitch = 0.0;
  my_controller->yaw = 0.0;
}

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE]){
  // Complementary Filter
  float accel_roll = atan2f(a[1], a[2]) * 180.0 / PI;
  float accel_pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * 180.0 / PI;

  float gyro_roll = w[0] * my_controller->dT;
  float gyro_pitch = w[1] * my_controller->dT;
  float gyro_yaw = w[2] * my_controller->dT;

  my_controller->roll = my_controller->com_alpha * (my_controller->roll + gyro_roll) + (1 - my_controller->com_alpha) * accel_roll;
  my_controller->pitch = my_controller->com_alpha * (my_controller->pitch + gyro_pitch) + (1 - my_controller->com_alpha) * accel_pitch;
  my_controller->yaw = (my_controller->yaw + gyro_yaw);

}