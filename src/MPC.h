#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
    
    struct solution {
        double d;
        double a;
        vector<double> x;
        vector<double> y;
    };
    
    MPC();
    
    virtual ~MPC();
    
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
