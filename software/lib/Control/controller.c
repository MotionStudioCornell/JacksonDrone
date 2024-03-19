// controller.c
#include "controller.h"

void init_controller(controller *my_controller, float alpha, float dT, float Kp, float Ki, float Kd)
{
  my_controller->com_alpha = alpha;
  my_controller->dT = dT;
  my_controller->Kp = Kp;
  my_controller->Ki = Ki;
  my_controller->Kd = Kd;

  // setting body frame the reference frame
  my_controller->x.roll = 0.0f;
  my_controller->x.pitch = 0.0f;
  my_controller->x.yaw = 0.0f;

  my_controller->x.d_roll = 0.0f;
  my_controller->x.d_pitch = 0.0f;
  my_controller->x.d_yaw = 0.0f;
}

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE])
{
  // Complementary Filter
  float accel_roll = atan2f(a[1], a[2]) * 180.0 / PI;
  float accel_pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * 180.0 / PI;

  float gyro_roll = w[0] * my_controller->dT;
  float gyro_pitch = w[1] * my_controller->dT;
  float gyro_yaw = w[2] * my_controller->dT;

  float roll_new = my_controller->com_alpha * (my_controller->x.roll + gyro_roll) + (1 - my_controller->com_alpha) * accel_roll;
  float pitch_new = my_controller->com_alpha * (my_controller->x.pitch + gyro_pitch) + (1 - my_controller->com_alpha) * accel_pitch;
  float yaw_new = (my_controller->x.yaw + gyro_yaw);
  //--------------------------------------------
  // degree/sec
  float d_roll_new = (roll_new - my_controller->x.roll) / my_controller->dT;
  float d_pitch_new = (pitch_new - my_controller->x.pitch) / my_controller->dT;
  float d_yaw_new = (yaw_new - my_controller->x.yaw) / my_controller->dT;
  // degree/sec^2
  float d2_roll_new = d_roll_new - my_controller->dx.d_roll;
  float d2_pitch_new = d_pitch_new - my_controller->dx.d_pitch;
  float d2_yaw_new = d_pitch_new - my_controller->dx.d_yaw;

  // Update the state vector x and dx
  my_controller->x.roll = roll_new;
  my_controller->x.pitch = pitch_new;
  my_controller->x.yaw = yaw_new;

  my_controller->x.d_roll = d_roll_new;
  my_controller->x.d_pitch = d_pitch_new;
  my_controller->x.d_yaw = d_yaw_new;
  my_controller->dx.d_roll = d_roll_new;
  my_controller->dx.d_pitch = d_pitch_new;
  my_controller->dx.d_yaw = d_yaw_new;

  my_controller->dx.d2_roll = d2_roll_new;
  my_controller->dx.d2_pitch = d2_pitch_new;
  my_controller->dx.d2_yaw = d2_yaw_new;

  //--------------------------------------------
}