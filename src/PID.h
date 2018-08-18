#include <vector>

#ifndef PID_H
#define PID_H

class PID {
public:
  /*
   * constants
   */
  const int WINDOW_SIZE = 50;

  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double total_cte;
  long long count;
  double best_average;
  double ctes[100];

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Differentials for twiddle algorithm
   */
  double dp;
  double di;
  double dd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double dp, double di, double dd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Calculate the average error.
   */
  double AverageError();

  double Run(double cte);
};

#endif /* PID_H */
