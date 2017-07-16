#ifndef PID_H
#define PID_H

#include <vector>


class PID {
public:
  // Errors
  double p_error;
  double i_error;
  double d_error;
  // Coefficients
  std::vector<double> Kpid;


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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};



class Twiddle {
public:
  /*
  * Coefficients
  */
  std::vector<double> dp_;
  // Twiddle params
  int cnt;
  int idx;
  int attempt;
  double best_err;
  double cur_err;

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize Twiddle.
  */
  void Init(double dKp, double dKi, double dKd);

  /*
  * Update the twiddle state given the integrated cross track error.
  */
  void NewParams(double& i_error, std::vector<double>& Kpid);
};

#endif /* PID_H */
