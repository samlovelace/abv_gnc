#ifndef ABVSTATE_HPP
#define ABVSTATE_HPP

#include <vector> 
#include <eigen3/Eigen/Dense>

struct AbvState
{
    double x, y, theta;
    double vx, vy, omega;

    AbvState operator-(const AbvState& other) const {
        return {
            x - other.x,
            y - other.y,
            theta - other.theta,
            vx - other.vx,
            vy - other.vy,
            omega - other.omega
        };
    }
};

inline std::vector<double> toVector(const AbvState& x) 
{ 
    return { x.x, x.y, x.theta, x.vx, x.vy, x.omega }; 
}

inline Eigen::Matrix<double, 6, 1> toEigen(const AbvState& x)
{
    Eigen::Matrix<double, 6, 1> state; 
    state << x.x, x.y, x.theta, x.vx, x.vy, x.omega; 

    return state; 
}

inline AbvState fromEigen(const Eigen::Matrix<double, 6, 1>& aState)
{
    AbvState state; 
    state.x = aState[0]; 
    state.y = aState[1]; 
    state.theta = aState[2]; 

    state.vx = aState[3]; 
    state.vy = aState[4]; 
    state.omega = aState[5]; 

    return state; 
}


#endif 
