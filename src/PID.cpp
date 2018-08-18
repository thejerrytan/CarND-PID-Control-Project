#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double dp, double di, double dd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->dp = dp;
  this->di = di;
  this->dd = dd;
  this->count = 0;
  this->prev_cte = 0;
  this->total_cte = 0;
  this->best_average = 999999;
}

void PID::UpdateError(double cte) {
  this->ctes[count % WINDOW_SIZE] = cte;
  this->total_cte += cte; // cte can be -ve or +ve
  this->prev_cte = cte;
  this->count += 1;
}

double PID::TotalError() {
  return this->total_cte;
}

double PID::AverageError() {
  double totalError = 0;
  if (count < 100) { // ctes array haven't fill up yet
    for (int i = 0; i < count; ++i) { totalError += ctes[i]; }
    return fabs(totalError / count);
  } else {
    for (int i = 0; i < WINDOW_SIZE; ++i) { totalError += ctes[i]; }
    return fabs(totalError / WINDOW_SIZE);
  }
}

double PID::Run(double cte) {
  double p = -Kp * cte;
  double i = Ki * TotalError();
  double d = Kd * (cte - prev_cte);
  double steer_value = fmax(fmin(p - d - i, 1.0), -1.0);
  UpdateError(cte);
  return steer_value;
}