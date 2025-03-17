#include "control.h"


// ==================================================
// Integrator
// ==================================================

void Integrator::init(float time_sampling)
{
  if(time_sampling > 0.0) {
    this->ts = time_sampling;
  } else {
    this->ts = 1.0;
  }

  reset();
}

void Integrator::reset()
{
  reset(0.0);
}

void Integrator::reset(float x)
{
  this->x = x;
}

void Integrator::input(float u)
{
  this->u = u;
}

void Integrator::step()
{
  x = x + ts*u;
}

float Integrator::output()
{
  return x;
}

float Integrator::evolve(float u)
{ 
  float y;

  input(u);
  y = output();
  step();

  return y;
}


// ==================================================
// Filter
// ==================================================

void Filter::init(float time_sampling, float tau)
{
  float ts = time_sampling;
  
  if(ts > 0.0) {
    A = exp(-time_sampling/tau);
    B = (1.0-A)*tau;
    C = 1.0/tau;
  } else {
    A = exp(-1.0/tau);
    B = 1.0-A;
    C = 1.0;
  }

  reset();
}

void Filter::reset()
{
  reset(0.0);
}

void Filter::reset(float x)
{
  this->x = x;
}

void Filter::input(float u)
{
  this->u = u;
}

void Filter::step()
{
  x = A*x + B*u;
}

float Filter::output()
{
  return C*x;
}

float Filter::evolve(float u)
{
  float y;
  
  input(u);
  y = output();
  step();

  return y;
}


// ==================================================
// PID
// ==================================================

void PID::init(float time_sampling, 
  float err_deadzone, 
  float int_sat, float int_reset_err_thr, float int_reset_div, float int_reset_val, 
  float der_filter_pole, 
  bool bumpless)
{
  this->ts = time_sampling;
  this->err_deadzone = err_deadzone;
  this->int_sat = int_sat;
  this->int_rst_thr = int_reset_err_thr;
  this->int_rst_div = int_reset_div;
  this->int_rst_val = int_reset_val;
  this->der_pole = der_filter_pole;
  this->bumpless = bumpless;

  if(ts > 0.0) {
    if(der_pole > 0.0) {
      A = exp(-der_pole*ts);
      B = (1.0-A)/der_pole;
      C = -der_pole*der_pole;
      D = der_pole;
    } else {
      A = 0.0;
      B = 1.0;
      C = -1.0/ts;
      D = +1.0/ts;
    }
  } else {
    ts = 1.0;
    if(der_pole > 0.0) {
      A = exp(-der_pole);
      B = 1.0-A;
      C = -B;
      D = +B;
    } else {
      A = 0.0;
      B = 1.0;
      C = -B;
      D = +B;
    }
  }

  reset();
}

void PID::setup(float kp, float ki, float kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::reset()
{
  reset(0.0, 0.0);
}

void PID::reset(float xi, float xd)
{
  this-> xi = xi;
  this-> xd = xd;

  apply_saturation();
}

void PID::input(float e)
{
  this->e = e;
}

void PID::step()
{
  float e = (fabs(this->e) < err_deadzone) ? 0.0 : this->e;

  xi = xi + (bumpless ? ki * ts * e : ts * e);
  xd = A * xd + (bumpless ? kd * B * e : B * e);

  if(int_rst_thr > 0.0 && fabs(e) > int_rst_thr && (e * xi) < 0.0) {
    xi /= int_rst_div;
    if(fabs(xi) > int_rst_val) {
      xi = (xi < 0) ? -int_rst_val : int_rst_val;
    }
  }

  apply_saturation();
}

float PID::output()
{
  float u;

  float e = (fabs(this->e) < err_deadzone) ? 0.0 : this->e;

  if(bumpless) u = (kp + kd*D) * e + xi + C*xd;
  else u = (kp + kd*D) * e + ki*xi + kd*C*xd;

  return u;
}

float PID::evolve(float e)
{ 
  float u;
  
  input(e);
  u = output();
  step();

  return u;
}

void PID::apply_saturation()
{
  if(int_sat > 0.0 && fabs(xi) > int_sat) {
    xi = (xi < 0.0) ? -int_sat : int_sat;
  }
}