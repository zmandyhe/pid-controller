#ifndef PID_H
#define PID_H
#include <vector>


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle variables
  */
  std::vector<double> dp;
  int step, p_index;
  int n_settle_steps, n_eval_steps;
  double total_error, best_error;
  bool first, second;

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

  /*
  * function for adding amount (dp) to a PID controller parameter based on index
  */
  void Cal_Parameter(int index, double dp_value);
  
  /*
  * function for calculating sum of dp
  */
  double GetSumdp();
};

#endif /* PID_H */
