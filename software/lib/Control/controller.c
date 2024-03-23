// controller.c
#include "controller.h"

void set_state(controller *my_controller, float roll, float pitch, float yaw, float droll, float dpitch, float dyaw)
{

  my_controller->x.roll = 0.0f;
  my_controller->x.pitch = 0.0f;
  my_controller->x.yaw = 0.0f;

  my_controller->x.d_roll = 0.0f;
  my_controller->x.d_pitch = 0.0f;
  my_controller->x.d_yaw = 0.0f;
}

void set_throttle(controller *my_controller, float t1, float t2, float t3, float t4)
{
  my_controller->u.t1 = t1;
  my_controller->u.t2 = t2;
  my_controller->u.t3 = t3;
  my_controller->u.t4 = t4;
}

state get_state_diff(state x, state target)
{
  state diff;

  diff.roll = target.roll - x.roll;
  diff.pitch = target.pitch - x.pitch;
  diff.yaw = target.yaw - x.yaw;

  diff.d_roll = target.d_roll - x.d_roll;
  diff.d_pitch = target.d_pitch - x.d_pitch;
  diff.d_yaw = target.d_yaw - x.d_yaw;

  return diff;
}

void print_state(state x)
{
  printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, dRoll: %.2f, dPitch: %.2f, dYaw: %.2f \n", x.roll, x.pitch, x.yaw, x.d_roll, x.d_pitch);
}

void print_control(control u)
{
  printf("t1: %.2f, t2: %.2f, t3: %.2f, t4: %.2f\n", u.t1, u.t2, u.t3, u.t4);
}

void init_controller(controller *my_controller, float alpha, float dT, float Kp, float Ki, float Kd)
{
  my_controller->com_alpha = alpha;
  my_controller->dT = dT;
  my_controller->Kp = Kp;
  my_controller->Ki = Ki;
  my_controller->Kd = Kd;

  set_throttle(my_controller, 0.0f, 0.0f, 0.0f, 0.0f);
  // setting body frame the reference frame
  set_state(my_controller, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void update_u(controller *my_controller)
{
  state diff = get_state_diff(my_controller->x, my_controller->target);

  // Roll
  float roll_P = my_controller->Kp * (diff.roll);

  my_controller->u.t1 = roll_P / 2;
  my_controller->u.t2 = roll_P / 2;

  my_controller->u.t3 = -roll_P / 2;
  my_controller->u.t4 = -roll_P / 2;

  print_control(my_controller->u);
}

void update_complementry_filter(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE])
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

  // Update the state vector x and dx
  my_controller->x.roll = roll_new;
  my_controller->x.pitch = pitch_new;
  my_controller->x.yaw = yaw_new;

  my_controller->x.d_roll = d_roll_new;
  my_controller->x.d_pitch = d_pitch_new;
  my_controller->x.d_yaw = d_yaw_new;

  //--------------------------------------------
}

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE])
{
  update_complementry_filter(my_controller, w, a);
  update_u(my_controller);
}