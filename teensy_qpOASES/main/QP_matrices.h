// Parameters for online QP solving

#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

#ifndef QP_matrices
#define QP_matrices

// EDIT - Problem dimensions
#define MPC_n    // Number of states
#define MPC_m    // Number of inputs
#define MPC_c    // Number of constraints


// EDIT - cost
real_t QP_H[MPC_m*MPC_m] = {}; 

real_t QP_F[MPC_n*MPC_m] = {}; 

// EDIT - constraints
real_t QP_G[MPC_c*MPC_m] = {};

real_t QP_w[MPC_c] = {}; 

real_t QP_E[MPC_c*MPC_n] = {}; 


real_t QP_lb[MPC_m] = {};
real_t QP_ub[MPC_m] = {};
real_t QP_lbA[MPC_c] = {};

#endif
