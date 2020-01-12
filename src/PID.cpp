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
  this->iter = 0;
  this->accum_error = 0;
  this->average_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, int sample_size)
{
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  p_error = 0;
  d_error = 0;
  i_error = 0;
  sample_size_ = sample_size;
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

double PID::getAveTps()
{
  return count_ / difftime(time(NULL), init_time_);
  ;
}

void PID::update_params(double cte, double speed, double throttle)
{
  best_cte_ = fmin(fabs(cte), best_cte_);
  worst_cte_ = fmax(fabs(cte), worst_cte_);
  total_cte_ += fabs(cte);

  best_speed_ = fmax(speed, best_speed_);
  total_speed_ += speed;

  total_throttle_ += throttle;

  if (count_ == 0)
  {
    time(&init_time_);
  }
  if ((++count_ % sample_size_) == 0)
  {
    ave_cte_ = total_cte_ / count_;
    ave_speed_ = total_speed_ / count_;
    ave_throttle_ = total_throttle_ / count_;
    printf("best_cte_=%.5f, worst_cte_=%.5f, ave_cte=%.5f\n", best_cte_, worst_cte_, ave_cte_);
    printf("best_speed_=%.2f, ave_speed_=%.2f, ave_throttle_=%.3f, ave_tps_=%.1f\n", best_speed_, ave_speed_, ave_throttle_, getAveTps());
  }
}