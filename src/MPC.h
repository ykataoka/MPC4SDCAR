#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Set the timestep length and duration
#define NUM_STEP 20
#define DT 0.1

// Set the reference velocity
#define REF_V 26.66 // 60mph = 26.66m/s

// This is the length from front to CoG that has a similar radius.
#define LF 2.67

// weights for cost functions.
#define W_CTE 1
#define W_EPSI 1
#define W_V 1
#define W_DELTA 1000
#define W_A 1
#define W_DDELTA 10
#define W_DA 1

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // for visualization of MPC control prediction
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
