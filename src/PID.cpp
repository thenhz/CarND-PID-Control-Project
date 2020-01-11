#include "PID.h"
#include <vector>

using std::vector;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  	this->iter = 0;
	this->accum_error = 0;
	this->average_error = 0; 
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

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
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0; // TODO: Add your total error calc here!
}

double PID::getSteerValue()
{
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}