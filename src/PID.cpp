#include "PID.h"

using namespace std;
using std::vector;

/*
* PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kpid.push_back(Kp);
  Kpid.push_back(Ki);
  Kpid.push_back(Kd);
  i_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -(Kpid[0]*p_error + Kpid[1]*i_error + Kpid[2]*d_error);
}


/*
* Twiddle class.
*/

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double dKp_, double dKi_, double dKd_) {

  dp_.push_back(dKp_);
  dp_.push_back(dKi_);
  dp_.push_back(dKd_);

  cnt = 0;
  idx = 0;
  best_err = 1e8;
  cur_err = 0.0;
  attempt = 0;
}


void Twiddle::NewParams(double& cte, std::vector<double>& Kpid) {

  cur_err += cte*cte;

  if (best_err == 1e8 and cnt == 1000) {
    // This is the first twiddle iteration
    // Set error for initial params
    best_err = cur_err;
    cnt = 0;
    cur_err = 0.0;
    // Forward attempt for first variable
    Kpid[idx] += dp_[idx];
    return;
  }

  if (cnt == 100 and best_err != 1e8) {
    // Enough time has elapsed since last change. Gather error sum and
    // check how the new parameters performed
    cnt = 0;
    double err = cur_err;
    cur_err = 0.0;

    if (attempt == 0) {
      if (err < best_err) {
        // Good move, increase step size for this variable
        best_err = err;
        dp_[idx] *= 1.1;
        // Move to next index of PID
        idx = (idx + 1) % 3;
        // Forward attempt for next variable
        Kpid[idx] += dp_[idx];
        attempt = 0;
      } else {
        // Error increased, try backwards attempt for current variable
        Kpid[idx] -= 2*dp_[idx];
        attempt = 1;
      }
      return;
    }

    if (attempt == 1) {
      if (err < best_err) {
        best_err = err;
        dp_[idx] *= 1.1;
      } else {
      // Restore current variable
        Kpid[idx] += dp_[idx];
        dp_[idx] *= 0.9;
      }
      // Move to next index of PID. SKIP "I" of PID
      idx = (idx + 1) % 3;
      // Forward attempt for next variable
      Kpid[idx] += dp_[idx];
      attempt = 0;
      return;
    }

  } else {
    cnt += 1;
    return;
  }
}
