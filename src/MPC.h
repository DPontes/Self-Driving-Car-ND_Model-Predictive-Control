#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <chrono>

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  double ave_latency_ms_;   // stored average estimated latency
  double Lf_;               // distance from car's front wheels to CG for turning radius
  double px_pred_;          // car's x position predicted after latency
  double py_pred_;          // car's y position predicted after latency
  double psi_pred_;         // car's velocity predicted after latency
  double v_pred_;           // car's velocity predicted after latency

  std::vector<double> mpc_path_x_;  // MPC's planned path x coordinates
  std::vector<double> mpc_path_y_;  // MPC's planned path y coordinates

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
