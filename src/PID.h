#include <ctime>

#ifndef PID_H
#define PID_H

class PID
{
public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, int sample_size);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double getAveTps();

  void update_params(double cte, double speed, double throttle);

private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Iterations total count and error variables
  * accum_error is for monitoring purposes (could be reset), separate from the PID system internal variables.
  */
  long iter;
  double accum_error;
  double average_error;

  int sample_size_;
  int count_ = 0;

  // CTE tracking
  double best_cte_ = 1.0;
  double worst_cte_ = 0.0;
  double total_cte_ = 0.0;
  double ave_cte_ = 0.0;

  // Throttle tracking
  double ave_throttle_ = 0.0;
  double total_throttle_ = 0.0;

  // Speed tracking
  double best_speed_ = 0.0;
  double ave_speed_ = 0.0;
  double total_speed_ = 0.0;

  // Throughput tracking
  time_t init_time_;
  double ave_tps_ = 0.0;
};

#endif // PID_H