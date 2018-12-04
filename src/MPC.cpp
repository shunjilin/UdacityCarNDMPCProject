#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;
// TODO: Set the timestep length and duration
size_t const N = 15;
double const dt = 0.05;

double const ref_cte = 0;
double const ref_epsi = 0;
double const ref_v = 100;

// vars is a flat array
// below are the indexes of each var for timestep 0
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
      
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Cost function
        // TODO: Define the cost related the reference state and
        // any anything you think may be beneficial.

        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += 2500* CppAD::pow(vars[cte_start + t] - ref_cte, 2);
            fg[0] += 2500* CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += 100*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 100*CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        // Initialize model to  initial state
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        for (int t = 0; t < N-1; t++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t + 1];
            AD<double> y1 = vars[y_start + t + 1];
            AD<double> psi1 = vars[psi_start + t + 1];
            AD<double> v1 = vars[v_start + t + 1];
            AD<double> cte1 = vars[cte_start + t + 1];
            AD<double> epsi1 = vars[epsi_start + t + 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + t];
            AD<double> y0 = vars[y_start + t];
            AD<double> psi0 = vars[psi_start + t];
            AD<double> v0 = vars[v_start + t];
            AD<double> cte0 = vars[cte_start + t];
            AD<double> epsi0 = vars[epsi_start + t];

            AD<double> delta0 = vars[delta_start + t];
            AD<double> a0 = vars[a_start + t];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
            AD<double> psides0 = CppAD::atan(3*coeffs[3] * pow(x0, 2) + 2*coeffs[2] * x0 + coeffs[1]);

            
            fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + t] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + t] =
                epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
//MPC::MPC() {}
//MPC::~MPC() {}

// TODO: Set the number of model variables (includes both states and inputs).
// For example: If the state is a 4 element vector, the actuators is a 2
// element vector and there are 10 timesteps. The number of variables is:
//
// 4 * 10 + 2 * 9
// model variables and constraints
size_t const n_vars = n_state_vars * N + n_actuator_vars * (N-1);
size_t const n_constraints = n_state_vars * N;
double const  delta_bound_abs = 0.436332*Lf; // 25 degrees in radians
double const bound_abs = 1.0e19;
double const a_bound_abs = 1.0;

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];


    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // TODO: Set lower and upper limits for variables.

    // no limits for non actuator variables
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -bound_abs;
        vars_upperbound[i] = bound_abs;
    }

    // set steering constraints
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -delta_bound_abs;
        vars_upperbound[i] = delta_bound_abs;
    }

    // set acceleration constraings
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -a_bound_abs;
        vars_upperbound[i] = a_bound_abs;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;


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

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.

    vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (int i = 0; i < N-1; i++) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    
    return result;
}
