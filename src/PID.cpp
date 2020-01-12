#include "PID.h"
#include <vector>
#include <cmath>
#include <iostream>

using std::vector;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  p_error = 0;
  d_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError()
{
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}