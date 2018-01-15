#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// MPC solver number of timesteps and duration of each step
constexpr size_t kNSteps = 8;     // number of state timesteps
constexpr double kDeltaT = 0.15;  // time per step (sec)

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double kLf = 2.67;  // m

// Target velocity for cost function
// - Boosted to 115mph to have some margin for cost function to achieve
//   actual 100mph peak speed on lake track
constexpr double kRefVMax = 115;          // mph
constexpr double kRefVTurnFactor = 0.2;   // tuned factor to reduce speed in turns

// Set index points for varaible starting locations in the combined vector
const size_t x_start     = 0;
const size_t y_start     = x_start     + kNSteps;
const size_t psi_start   = y_start     + kNSteps;
const size_t v_start     = psi_start   + kNSteps;
const size_t cte_start   = v_start     + kNSteps;
const size_t epsi_start  = cte_start   + kNSteps;
const size_t delta_start = epsi_start  + kNSteps;
const size_t a_start     = delta_start + (kNSteps - 1); // N-1 delta actuators values

/*
  The FG_eval object sets up the MPC's cost function and constraints to be
  used as an input to the IPOPT optimizer for solving the time steps by the
  CppAD::ipopt::solve() method.
*/
class FG_eval {
 public:
  // Fitted waypoint polynomial coefficients
  Eigen::VectorXd coeffs;

  // Constructor
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  // ADvector typedef for convenience
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  /*
    The operator() method uses 'vars' CppAD vector of state variables
    to calculate the 'fg' vector that contains the cost at index 0,
    and the state constraints for each timestep at index 1~(kNSteps+1)
    based on the motion model differential equations with the form
    x1 = f(x0) -> 0 = x1 - f(x0).
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    /*
      Define cost function to be minimized

      Estimate endpoint of planned path poly coefficients after kNSteps
      of kDeltaT (x = v*t, y = f(x)). Calculate the angle from the car's
      current heading (0 degrees) to the endpoint of the planned path to use
      as a cost for difference from reference angle to be able to pull the
      path straighter across and cut corners in sharp turns to achieve higher
      speed and a better driving line
    */
    const CppAD::AD<double> x_end = vars[v_start] * (kNSteps * kDeltaT);
    const CppAD::AD<double> y_end = coeffs(0) +
                                    coeffs(1) * x_end +
                                    coeffs(2) * x_end * x_end +
                                    coeffs(3) * x_end * x_end * x_end;

    const CppAD::AD<double> angle_end = CppAD::atan2(y_end, x_end);

    /*
      Adjust reference speed to slow down in sharp turns using simple
      correlation to the lateral distance y_end of the planned path's
      endpoint and convert from mph -> mps.
    */
    const CppAD::AD<double> ref_v_by_turn =
                              (kRefVMax - kRefVTurnFactor * CppAD::abs(y_end))
                              * (1609.34 / 3600);

    // Initialize cost to zero before adding up each cost term
    fg[0] = 0;

    /*
      Add cost terms based on the reference state (cost term coefficients
    were manually tuned to achieve ~110mph peak speed with smooth driving)
    */
    for(unsigned int t = 0; t < kNSteps; t++){
      // Basic CTE
      fg[0] += 20 * CppAD::pow(vars[epsi_start + t], 2) * t;

      // Basic EPSI (progressive cost increases linearly by timesteps)
      fg[0] += 32000 * CppAD::pow(vars[epsi_start + t], 2) * t;

      // Difference from target reference velocity
      fg[0] += 1500 * CppAD::pow(vars[v_start + t] - ref_v_by_turn, 2);

      // Difference from angle to end point of horizon
      // (progressive cost increases linearny by timestep)
      fg[0] += 31000 * CppAD::pow(angle_end - vars[psi_start + t], 2) * t;
    }

    // Add cost terms to minimize the use of actuators
    for(unsigned int t = 0; t < kNSteps; t++){
      // Absolute steering angle
      fg[0] += 200000 * CppAD::pow(vars[delta_start + t], 2);
    }

    // Add cost terms to minimize the change between sequential actuations
    for(unsigned int t = 0; t < kNSteps - 2; t++){
      // Change in sequential steering actuations
      fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] -
                                vars[delta_start + t], 2);

      // Change in sequential throttle (acceleration) actuations
      fg[0] += 5000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /*
      Setup constraints
      Constraints at initial timestep set to initial state values. Starting
      indices of the 'fg' vector are incremented +1 to shift to after the Cost
      value stored at index 0.
    */
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Constraints at each time step set to follow motion model equations
    for(unsigned int t = 1; t < kNSteps; t++){
      // The state at time t1
      const CppAD::AD<double> x1    = vars[x_start + t];
      const CppAD::AD<double> y1    = vars[y_start + t];
      const CppAD::AD<double> psi1  = vars[psi_start + t];
      const CppAD::AD<double> v1    = vars[v_start + t];
      const CppAD::AD<double> cte1  = vars[cte_start + t];
      const CppAD::AD<double> epsi1 = vars[epsi_start + t];

      // The state at previous time t0
      const CppAD::AD<double> x0    = vars[x_start + t - 1];
      const CppAD::AD<double> y0    = vars[y_start + t - 1];
      const CppAD::AD<double> psi0  = vars[psi_start + t - 1];
      const CppAD::AD<double> v0    = vars[v_start + t - 1];
      const CppAD::AD<double> cte0  = vars[cte_start + t - 1];
      const CppAD::AD<double> epsi0 = vars[epsi_start + t - 1];

      // The actuation from time t0
      const CppAD::AD<double> delta0 = vars[delta_start + t - 1];
      const CppAD::AD<double> a0     = vars[a_start + t - 1];

      // f(x) = coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + coeffs[3]*x^3
      const CppAD::AD<double> f0 = coeffs(0) +
                                    coeffs(1) * x0 +
                                    coeffs(2) * (x0 * x0) +
                                    coeffs(3) * (x0 * x0 * x0);

      // psides = atan(f'(x))
      //        = atan(coeffs[1] + 2*coeffs[2]*x + 3*coeffs[3]*x^2)
      const CppAD::AD<double> psides0 = CppAD::atan(coeffs(1) + 2*coeffs(2)*x0
                                        + coeffs(3)*x0*x0);

      /*
        Equations for the motion model:
          x[t1]    = x[t0] + v[t0] * cos(psi[t0]) * dt
          y[t1]    = y[t0] + v[t0] * sin(psi[t0]) * dt
          psi[t1]  = psi[t0] + v[t0] / Lf * delta[t0] * dt;
          v[t1]    = v[t0] + a[t0] * dt
          cte[t1]  = (y[t0] - f(x[t0])) + (v[t0] * sin(epsi[t0]) * dt)
          epsi[t1] = (psi[t0] - psides[t0]) + (v[t0] * delta[t0] / Lf *dt)
       */
       fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * kDeltaT);
       fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * kDeltaT);
       fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / kLf * kDeltaT);
       fg[1 + v_start + t] = v1 - (v0 + a0 * kDeltaT);
       fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * kDeltaT));
       fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0 * delta0 / kLf * kDeltaT));
    }
  }
}; // class FG_eval

//
// MPC class definition implementation.
//
MPC::MPC() { Lf_ = kLf; }
MPC::~MPC() {}  // Destructor

/*
  The Solve() method takes the current state vector and waypoint polyfit
  coefficients and optimizes a planned trajectory path and actuations to minimize
  a cost function while satisfying constraints on the state variables.
  The cost function and constraints are defined in the FG_eval object.

  Returns a vector of the steering and throttle actuations from the 1st timestep
  of the MPC's planned path. The planned path x,y coords are also stored to
  the mpc_path_x_, mpc_path_y_ variables for visualization.
*/
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;

  typedef CPPAD_TESTVECTOR(double) Dvector;

  /*
    Set the number of constraints and varaible (state and actuator inputs)
    over the predicted horizon's timesteps.
    State variables are (x, y, psi, v, cte, epsi)
    Actuators are: steering delta, throttle acceleration
  */
  const size_t n_constraints = kNSteps * 6;         // state steps * 6 states
  const size_t n_actuators   = (kNSteps - 1) * 2;   // actuation steps * 2 actuators
  const size_t n_vars        = n_constraints + n_actuators;

  // Initial value of the independent variables. SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial state to current state vector values
  vars[x_start]    = state[0];
  vars[y_start]    = state[1];
  vars[psi_start]  = state[2];
  vars[v_start]    = state[3];
  vars[cte_start]  = state[4];
  vars[epsi_start] = state[5];

  // Set lower/upper limits for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for(unsigned int i = 0; i < delta_start; i++){
    vars_lowerbound = -1.0e19;    // Initialize all lower bounds to no limit
    vars_upperbound =  1.0e19;    // Initialize all upper bounds to no limit
  }

  // Limit steering delta to stay within [-25,25] degrees range
  for(unsigned int i = delta_start; i < a_start; i++){
    vars_lowerbound = -0.436332;    // -25degrees -> rad
    vars_upperbound =  0.436332;    // +25degrees -> rad
  }

  // Limit throttle acceleration to stay within [-1, +1] range
  for(unsigned int i = a_start; i < n_vars; i++){
    vars_lowerbound = -1.0;
    vars_upperbound =  1.0;
  }

  // Set lower/upper limits for the constraints. SHOULD BE 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set the initial state lower limits to current state vector values
  constraints_lowerbound[x_start]    = state[0];
  constraints_lowerbound[y_start]    = state[1];
  constraints_lowerbound[psi_start]  = state[2];
  constraints_lowerbound[v_start]    = state[3];
  constraints_lowerbound[cte_start]  = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  // Set the initial state upper limits to current state vector values
  constraints_upperbound[x_start]    = state[0];
  constraints_upperbound[y_start]    = state[1];
  constraints_upperbound[psi_start]  = state[2];
  constraints_upperbound[v_start]    = state[3];
  constraints_upperbound[cte_start]  = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // pack vector of MPC planned path x,y coordinates for visualization
  mpc_path_x_.clear();
  mpc_path_y_.clear();
  for(unsigned int i = 1; i < kNSteps; i++){
    mpc_path_x_.push_back(solution.x[x_start + i]);   // x from 2nd step on
    mpc_path_y_.push_back(solution.x[y_start + i]);   // y from 2nd step on
  }

  // Pack vector of the 1st step actuations for steering and throotle commands
  std::vector<double> mpc_actuation(2);
  mpc_actuation[0] = solution.x[delta_start];   // 1st actuation steering
  mpc_actuation[1] = solution.x[a_start];       // 1st actuation throotle

  return mpc_actuation;
}
