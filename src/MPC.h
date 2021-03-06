#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// DONE: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.1;
const int num_missed_controls = 100/(dt*1000);


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
