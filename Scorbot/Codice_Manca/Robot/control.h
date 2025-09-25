#ifndef CONTROL_H
#define CONTROL_H


#include "math.h"


class Integrator final
{
public:
  Integrator() {}
  ~Integrator() {}

  void init(float time_sampling);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);
  
private:
  float ts;
  float x;
  float u;
};


class Filter final
{
public:
  void init(float time_sampling, float tau);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);

private:
  float u;
  float x;

  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
};


class PID final
{
public:
  void init(float time_sampling, 
    float err_deadzone, 
    float int_sat, float int_reset_err_thr, float int_reset_div, float int_reset_val, 
    float der_filter_pole, 
    bool bumpless);
  void setup(float kp, float ki, float kd);
  void reset();
  void reset(float xi, float xd);
  void input(float e);
  void step();
  float output();
  float evolve(float e);

private:
  void apply_saturation();

  float ts = 0.0;
  float err_deadzone;
  float int_sat;
  float int_rst_thr;
  float int_rst_div;
  float int_rst_val;
  float der_pole;
  bool bumpless = false;

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  
  float e = 0.0;
  float xi = 0.0;
  float xd = 0.0;
  
  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
  float D = 0.0;
};


#endif  // CONTROL_H