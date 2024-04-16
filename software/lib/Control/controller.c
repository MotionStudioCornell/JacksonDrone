// controller.c
#include "controller.h"

//helper functions
void set_state(controller *my_controller, float roll, float pitch, float yaw)
{

  my_controller->x.roll = 0.0f;
  my_controller->x.pitch = 0.0f;
  my_controller->x.yaw = 0.0f;

  // my_controller->x.d_roll = 0.0f;
  // my_controller->x.d_pitch = 0.0f;
  // my_controller->x.d_yaw = 0.0f;
}

state make_state(float roll, float pitch, float yaw)
{
  state result;
  result.roll = roll;
  result.pitch = pitch;
  result.yaw = yaw;
  return result;
  // my_controller->x.d_roll = 0.0f;
  // my_controller->x.d_pitch = 0.0f;
  // my_controller->x.d_yaw = 0.0f;
}

void set_throttle(controller *my_controller, float t1, float t2, float t3, float t4)
{
  my_controller->u.t1 = t1;
  my_controller->u.t2 = t2;
  my_controller->u.t3 = t3;
  my_controller->u.t4 = t4;
}

state get_state_error(state x, state target)
{
  state error;

  error.roll = target.roll - x.roll;
  error.pitch = target.pitch - x.pitch;
  error.yaw = target.yaw - x.yaw;

  // error.d_roll = target.d_roll - x.d_roll;
  // error.d_pitch = target.d_pitch - x.d_pitch;
  // error.d_yaw = target.d_yaw - x.d_yaw;

  return error;
}

state get_state_sum(state x, state target)
{
  state sum;

  sum.roll = target.roll + x.roll;
  sum.pitch = target.pitch + x.pitch;
  sum.yaw = target.yaw + x.yaw;

  // sum.d_roll = target.d_roll + x.d_roll;
  // sum.d_pitch = target.d_pitch + x.d_pitch;
  // sum.d_yaw = target.d_yaw + x.d_yaw;

  return sum;
}

state get_state_multi(state x, float a)
{
  state result;

  result.roll = a * x.roll;
  result.pitch = a * x.pitch;
  result.yaw = a * x.yaw;

  // result.d_roll = a * x.d_roll;
  // result.d_pitch = a * x.d_pitch;
  // result.d_yaw = a * x.d_yaw;

  return result;
}

state get_state_saturate(state x, float lower, float upper)
{
  state result;

  result.roll = saturate(x.roll, lower, upper);
  result.pitch = saturate(x.pitch, lower, upper);
  result.yaw = saturate(x.yaw, lower, upper);

  return result;
}

void print_state(state x)
{
  printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f \n", x.roll, x.pitch, x.yaw);
}

void print_control(control u)
{
  printf("t1: %.2f, t2: %.2f, t3: %.2f, t4: %.2f\n", u.t1, u.t2, u.t3, u.t4);
}

// main functions
void init_controller(controller *my_controller, float alpha, float dT, float Kp, float Ki, float Kd)
{
  my_controller->com_alpha = alpha;
  my_controller->dT = dT;
  my_controller->Kp = Kp;
  my_controller->Ki = Ki;
  my_controller->Kd = Kd;

  set_throttle(my_controller, 0.0f, 0.0f, 0.0f, 0.0f);
  // setting body frame the reference frame
  set_state(my_controller, 0.0f, 0.0f, 0.0f);
}

void saturate_control(control *u, float cap)
{
  u->t1 = saturate(u->t1, -cap, cap);
  u->t2 = saturate(u->t2, -cap, cap);
  u->t3 = saturate(u->t3, -cap, cap);
  u->t4 = saturate(u->t4, -cap, cap);
}

void update_u(controller *my_controller, float w[VECTOR_SIZE])
{
  state error = get_state_error(my_controller->x, my_controller->target);
  // can just use gyro data here or state error
  // state d_error = get_state_error(my_controller->prev_error, error);

  // use gyro
  state d_error = make_state(-w[0], -w[1], -w[2]);

  my_controller->integral_error = (my_controller->integral_error, get_state_multi(error, my_controller->dT));
  my_controller->integral_error = get_state_saturate(my_controller->integral_error, -Integral_error_sat, Integral_error_sat);

  // if we use state error not gyro
  my_controller->prev_error = error;

  float motor1 = 0.0f;
  float motor2 = 0.0f;
  float motor3 = 0.0f;
  float motor4 = 0.0f;
  // Roll
  float roll_P = my_controller->Kp * (error.roll);
  float roll_I = my_controller->Ki * (my_controller->integral_error.roll);
  float roll_D = my_controller->Kd * (d_error.roll);

  motor1 += -roll_P - roll_I - roll_D;
  motor2 += -roll_P - roll_I - roll_D;

  motor3 += roll_P + roll_I + roll_D;
  motor4 += roll_P + roll_I + roll_D;

  // Pitch
  float pitch_P = my_controller->Kp * (error.pitch);
  float pitch_I = my_controller->Ki * (my_controller->integral_error.pitch);
  float pitch_D = my_controller->Kd * (d_error.pitch);

  motor2 += -pitch_P - pitch_I - pitch_D;
  motor4 += -pitch_P - pitch_I - pitch_D;

  motor1 += pitch_P + pitch_I + pitch_D;
  motor3 += pitch_P + pitch_I + pitch_D;

  my_controller->u.t1 = motor1;
  my_controller->u.t2 = motor2;
  my_controller->u.t3 = motor3;
  my_controller->u.t4 = motor4;

  // saturate the control to [-cap, cap]
  saturate_control(&my_controller->u, 10);
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
  // float d_roll_new = (roll_new - my_controller->x.roll) / my_controller->dT;
  // float d_pitch_new = (pitch_new - my_controller->x.pitch) / my_controller->dT;
  // float d_yaw_new = (yaw_new - my_controller->x.yaw) / my_controller->dT;

  // Update the state vector x and dx
  my_controller->x.roll = roll_new;
  my_controller->x.pitch = pitch_new;
  my_controller->x.yaw = yaw_new;

  // my_controller->x.d_roll = d_roll_new;
  // my_controller->x.d_pitch = d_pitch_new;
  // my_controller->x.d_yaw = d_yaw_new;

  //--------------------------------------------
}

void update_controller(controller *my_controller, float w[VECTOR_SIZE], float a[VECTOR_SIZE])
{
  update_complementry_filter(my_controller, w, a);
  update_u(my_controller, w);
}