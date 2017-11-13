#include "MPC.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration to 1 second
size_t N = 10;
double dt = 0.03;
double delay_t = 0.100;

const double w_cte  = 50; // Enforce proximity to the trajectory
const double w_epsi = 5000; // Enforce proximity of the car rotation to the rotation implied by the trajectory
const double w_psi  = 1000;
const double w_d    = 5000; // Enforce smooth steering
const double w_tv   = 0.1; // Encourage speed

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
const double Lf = 2.67; // Udacity SDCND Lesson 18.3 Link - https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/a62153eb-458b-4c68-96fc-eeaa3b4aeaa6
const double mph2mps = 0.44704;
const double map_bounds = 1e5;
const double velocity_upper_bound = 150.00/mph2mps; // in mph
const double velocity_lower_bound = -20/mph2mps; // in mph
const double t_v = velocity_upper_bound * 0.6;
const double steering_max = 25*M_PI/180;

const size_t x_start = 0;
const size_t y_start = x_start+N;
const size_t psi_start = y_start+N;
const size_t v_start = psi_start+N;
const size_t d_start = v_start+N;
const size_t a_start = d_start+N-1;

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
        
        // Initialize state t=0
        AD<double> x0   = vars[x_start];
        AD<double> y0   = vars[y_start];
        AD<double> psi0 = vars[psi_start];
        AD<double> v0   = vars[v_start];
        AD<double> d1   = vars[d_start];
        AD<double> a1   = vars[a_start];
        AD<double> d0   = d1;
        AD<double> a0   = a1;
        
        // Initialize cost placeholder
        fg[0] = 0.0;
        // Setup state constraints at state t=0.
        // TODO: constraints at t=0 seem irrelevant. Test performance
        // without initial constraints
        fg[1+x_start]   = x0 - vars[x_start];
        fg[1+y_start]   = y0 - vars[y_start];
        fg[1+psi_start] = psi0 - vars[psi_start];
        fg[1+v_start]   = v0 - vars[v_start];
        
        for (int t=1; t<N; t++) {
            // Calculate x(t+1)
            AD<double> x1       = x0 + CppAD::cos(psi0)*(v0 * dt);
            fg[x_start+t]       = x1 - vars[x_start+t];
            // Calculate y(t+1)
            AD<double> y1       = y0 + CppAD::sin(psi0)*(v0 * dt);
            fg[y_start+t]       = y1 - vars[y_start+t];
            // Calculate trajectory f(coeff, x(t+1))
            AD<double> f1       = 0;
            for (size_t t_id = 0; t_id<coeffs.size(); t_id++) {
                f1+=coeffs[t_id]*CppAD::pow(x1, t_id);
            }
            // Calculate CTE
            AD<double> cte1     = f1 - y1;
            // Calculate psi(t+1)
            AD<double> psi1     = psi0 + v0/Lf * d1 * dt;
            fg[psi_start+t]     = psi1 - vars[psi_start+t];
            // Calculate velocity(t+1)
            AD<double> v1       = v0 + a1 * dt;
            fg[v_start+t]       = v1 - vars[v_start+t];
            // Calculate trajectory psi based on atan(coeff[1]):
            // ** Udacity SDCND, Class 19.9: AD<double> psides0 = CppAD::atan(coeffs[1]);
            AD<double> f_psi1   = 0; // the rotation of the car defined as a sloap of tangent line at x. The sloap of tangent line defined as a derivative of the floap.
            for (size_t t_id = 1; t_id<coeffs.size(); t_id++) {
                f_psi1 += double(t_id)*coeffs[t_id]*CppAD::pow(x1, t_id-1);
            }
            f_psi1   = CppAD::atan(f_psi1);
            // Calculate epsi(t+1)
            AD<double> epsi1    = f_psi1 - psi1;
            // Calculate total cost function
            fg[0] += w_cte  * CppAD::pow(cte1,      2); // Enforce proximity to the trajectory
            fg[0] += w_epsi * CppAD::pow(epsi1,     2) * v0/Lf; // Enforce proximity of the car rotation to the rotation implied by the trajectory
            fg[0] += w_psi  * CppAD::pow(psi1-psi0, 2) * v0/Lf;
            fg[0] += w_d    * CppAD::pow(d1,        2) * v0/Lf; // Enforce smooth steering
            fg[0] += w_tv   * CppAD::pow(t_v-v1,    2); // Encourage speed
            // Finish motion model cycle by setting d(t) and a(t)
            x0      = x1;
            y0      = y1;
            psi0    = psi1;
            v0      = v1;
            d0      = d1;
            a0      = a1;
            d1      = vars[d_start+t];
            a1      = vars[a_start+t];
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
    beginning_time = clock();
    step = 0;
}
MPC::~MPC() {}

MPC::solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    // Set the number of model variables (includes both states and inputs).
    
    // State representation includes 4 elements per timestep: [x,y,psi,v]
    // In addition, actuators include 2 elements per timestep [d, a]. Timestep starts at
    // t+1 as state representation include all information necessary. Prior controlls
    // do not introduce useful information.
    // Note: states starting at t+1 are not independent variables. Therefore,
    // they should not be included, and rather should be calculated by a motion model.
    // Therefore the number of independent variables is state_dim + control_dim * (N-1)
    size_t n_vars = 6*N;
    // Set the number of constraints
    size_t n_constraints = 4*N;
    
    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set lower and upper limits for variables.
    // Set up initial states
    vars_lowerbound[x_start]    = state[0];
    vars_upperbound[x_start]    = state[0];
    vars_lowerbound[y_start]    = state[1];
    vars_upperbound[y_start]    = state[1];
    vars_lowerbound[psi_start]  = state[2];
    vars_upperbound[psi_start]  = state[2];
    vars_lowerbound[v_start]    = state[3]; // in m/s
    vars_upperbound[v_start]    = state[3]; // in m/s
    // Set up other state bounds
    for (size_t t = 1; t<N; t++) {
        vars_lowerbound[x_start + t]    = -map_bounds;
        vars_upperbound[x_start + t]    = map_bounds;
        vars_lowerbound[y_start + t]    = -map_bounds;
        vars_upperbound[y_start + t]    = map_bounds;
        vars_lowerbound[psi_start + t]  = -M_PI;
        vars_upperbound[psi_start + t]  = M_PI;
        vars_lowerbound[v_start + t]    = velocity_lower_bound; // in m/s
        vars_upperbound[v_start + t]    = velocity_upper_bound; // in m/s
    }
    
    // Set up control bounds
    for (size_t t = 0; t<N-1; t++) {
        vars_lowerbound[d_start + t]    = -steering_max;
        vars_upperbound[d_start + t]    = steering_max;
        vars_lowerbound[a_start + t]    = -1.0;
        vars_upperbound[a_start + t]    = 1.0;
    }
    
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    // Set up state bounds that will ensure states equal to calculated states
    for (size_t t = 0; t<n_constraints; t++) {
        constraints_lowerbound[t]       = 0;
        constraints_upperbound[t]       = 0;
    }
    
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
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars,
                                          vars_lowerbound, vars_upperbound,
                                          constraints_lowerbound, constraints_upperbound,
                                          fg_eval, solution);
    
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    
    // Return the first actuator values. The variables can be accessed with
    MPC::solution s;
    s.d = solution.x[d_start];
    s.a = solution.x[a_start];
    for (size_t t=1; t<N; t++) {
        s.x.push_back(solution.x[x_start+t]);
        s.y.push_back(solution.x[y_start+t]);
    }
    // measure execution time
    step += 1;
    time_t end_time = clock();
    cout << "execution time (in miliseconds): " << (end_time - beginning_time)/step << endl;
    return s;
}
