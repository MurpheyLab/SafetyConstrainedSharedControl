//---------------------------------------------------------------------
// SCSC.hpp
// This header file contains ball-in-bowl and SCSC class definitions.
// The ball-in-bowl class defines the system dynamics and provides
// funtions for simulating the system forward in time.
//---------------------------------------------------------------------

#ifndef SCSC_HPP
#define SCSC_HPP


#include "rk4_int.hpp"
#include <cmath>
#include <iostream>
const double PI = 3.1415926535987;
using namespace std;

#include<Eigen>
//using namespace Eigen;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 2, 1> Vector2d;
typedef Matrix<double, 8, 8> Matrix8d;
typedef Matrix<double, 2, 2> Matrix2d;
typedef Matrix<double, 8, 2> Matrix82d;

#include <stdio.h>
//#include <stdlib.h>
//#include <time.h>
#include <random>
#include <chrono>
double rand_uniform_num(double a, double b)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    static std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(a, b);
    return distribution(generator);
}

//static std::default_random_engine generator;
//std::uniform_real_distribution<double> distribution(-1.0, 1.0);

//---------------------------------------------------------------------
//               B A L L - I N - B O W L   C L A S S
//---------------------------------------------------------------------
class BallinBowl {
public:
    double B;                  // damping
    double g;                  // gravity
    double m;                  // mass
    double h;                  // pendulum length
    double dt;                 // time between system updates
    double tcurr = 0.0;        // initilaize system time
    Vector8d Xcurr;            // current system state 
    Vector2d Ucurr;            // current control input
    // class member function prototypes   
    BallinBowl(double, double, double, double, double);
    Vector8d f(const Vector8d& x, const Vector2d& u);
    void step(void);
};

BallinBowl::BallinBowl(double _m, double _B, double _g, double _h, double _dt) {
    B = _B;     // damping
    g = _g;     // gravity
    m = _m;     // mass
    h = _h;     // pendulum length
    dt = _dt;   // step size
}

Vector8d BallinBowl::f(const Vector8d& x, const Vector2d& u) {                 //system dynamics, xdot = f(x)
    Vector8d xdot;
    xdot(0) = x(1);                                                            // xthetadot
    xdot(1) = (g / h * sin(x(0)) + B * x(1) / (m * h * h) - u(0) * cos(x(0))); // xthetadotdot
    xdot(2) = x(3);                                                            // ythetadot
    xdot(3) = (g / h * sin(x(2)) + B * x(3) / (m * h * h) - u(1) * cos(x(2))); // ythetadotdot
    xdot(4) = x(5);                                                            // bowlxdot
    xdot(5) = u(0);                                                            // bowlxdotdot
    xdot(6) = x(7);                                                            // bowlydot
    xdot(7) = u(1);                                                            // bowlydotdot      
    return xdot;
};

void BallinBowl::step() {                                                      //step the system foward by dt seconds
    Xcurr = RK4_step(this, Xcurr, Ucurr, dt);
    tcurr = tcurr + dt;
}; 



//---------------------------------------------------------------------
//                      i S A C   C L A S S
//---------------------------------------------------------------------
class iSAC {
private:
    double B;                  // damping
    double g;                  // gravity
    double m;                  // mass
    double h;                  // pendulum length
    double gamma = -0.75;      // controller parameters
    double beta = 0.1;           
public:
    double dt;                 // time between system updates (simulation timestep)
    double duration = 0.05;    // amount of time to apply a single control signal 
    double T = 0.5;            // how long the main iSAC loop will iterate  (horizon)
    double pred_horizon = 0.5; // length of horizon for a single iSAC iteration (size of chunks that iSAC considers at a time)
    double t_curr = 0.0;       // initilaize time
    double u_min, u_max;       // saturation limits
    double h_max;              // max ball height
    double epsilon = 0.01;
    Matrix2d R = Matrix2d::Identity();

    // class member function prototypes
    iSAC(BallinBowl, double, double, double);
    Vector8d f(const Vector8d& x, const Vector2d& u);
    Vector8d pdot(const Vector8d& p, const Vector8d& x, const Vector2d& u);
    Vector8d step_forward(const Vector8d& x, const Vector2d& u);
    Vector8d step_backward(const Vector8d& p, const Vector8d& x, const Vector2d& u);
    double cost(const MatrixXd& x, const MatrixXd& u);
    Vector2d predict(const Vector8d& x, const Vector2d& u);
};

iSAC::iSAC(BallinBowl sys, double min, double max, double bound) {
    B = sys.B;     // damping
    g = sys.g;     // gravity
    m = sys.m;     // mass
    h = sys.h;     // pendulum length
    dt = sys.dt;   // step size
    u_min = min;
    u_max = max;
    h_max = bound;
}

Vector8d iSAC::f(const Vector8d& x, const Vector2d& u) {                       //system dynamics, xdot = f(x)
    Vector8d xdot;
    xdot(0) = x(1);                                                            // xthetadot
    xdot(1) = (g / h * sin(x(0)) + B * x(1) / (m * h * h) - u(0) * cos(x(0))); // xthetadotdot
    xdot(2) = x(3);                                                            // ythetadot
    xdot(3) = (g / h * sin(x(2)) + B * x(3) / (m * h * h) - u(1) * cos(x(2))); // ythetadotdot
    xdot(4) = x(5);                                                            // bowlxdot
    xdot(5) = u(0);                                                            // bowlxdotdot
    xdot(6) = x(7);                                                            // bowlydot
    xdot(7) = u(1);                                                            // bowlydotdot      
    return xdot;
};

Vector8d iSAC::pdot(const Vector8d& p, const Vector8d& x, const Vector2d& u) { // costate variable dynamics 
    // linearization matrix
    Matrix8d A = Matrix8d::Zero();
    A(1, 0) = (g / h * cos(x(0))) + (u(0) * sin(x(0)));
    A(0, 1) = 1.0;
    A(1, 1) = B / (m * h * h);
    A(3, 2) = (g / h * cos(x(2))) + (u(1) * sin(x(2)));
    A(2, 3) = 1.0;
    A(4, 5) = B / (m * h * h);
    A(6, 7) = 1.0;
    // gradient of barrier function with respect to x (which is dldx)
    Vector8d dphidx = Vector8d::Zero();
    //dphidx(0) = epsilon * tan(x(0));
    //dphidx(2) = epsilon * tan(x(2));
    dphidx(0) = epsilon * h * (sin(x(0)) * cos(x(2)))/(h_max + h * ((cos(x(0)) * cos(x(2))) - 1));
    dphidx(2) = epsilon * h * (sin(x(2)) * cos(x(0)))/(h_max + h * ((cos(x(0)) * cos(x(2))) - 1));
    return A * p - dphidx;
};

Vector8d iSAC::step_forward(const Vector8d& x, const Vector2d& u) {
    Vector8d k1, k2, k3, k4, x_next;
    k1 = dt * f(x, u);
    k2 = dt * f(x + (0.5 * k1), u);
    k3 = dt * f(x + (0.5 * k2), u);
    k4 = dt * f(x + k3, u);
    x_next = x + ((1. / 6.) * k1) + ((1. / 3.) * k2) + ((1. / 3.) * k3) + ((1. / 6.) * k4);
    return x_next;
}

Vector8d iSAC::step_backward(const Vector8d& p, const Vector8d& x, const Vector2d& u) {
    Vector8d k1, k2, k3, k4, p_next;
    k1 = -dt * pdot(p, x, u);
    k2 = -dt * pdot(p + (0.5 * k1), x, u);
    k3 = -dt * pdot(p + (0.5 * k2), x, u);
    k4 = -dt * pdot(p + k3, x, u);
    p_next = p + ((1. / 6.) * k1) + ((1. / 3.) * k2) + ((1. / 3.) * k3) + ((1. / 6.) * k4);
    return p_next;
}

double iSAC::cost(const MatrixXd& x_traj, const MatrixXd& u_traj) {
    double J = 0.0;
    double phi, h_i, theta_x, theta_y;
    for (int i = 0; i < x_traj.cols(); i++)
    {
        h_i = h * (1 - (cos((x_traj(0, i))) * cos((x_traj(2, i))))) - h_max;
        if (h_i > 0)
        {
            phi = pow(10,5);
        } 
        else
        { 
            phi = -log(-h_i);
        }
        J = J + dt * ((0.5 * (u_traj.col(i)).transpose() * R * u_traj.col(i)) + (epsilon * phi));
    }
    return J;
}

Vector2d iSAC::predict(const Vector8d& x, const Vector2d& u) {
    // Initialize variables
    Vector2d u_send = u;
    //double pred_horizon = 0.5;
    //double T = 0.5;
    const int N = (int) ((T+pred_horizon)/dt) + 1;
    const int NN = (int) (pred_horizon/dt);
    VectorXd time = VectorXd::LinSpaced(N, t_curr, (t_curr + T + pred_horizon));
    MatrixXd x_traj = MatrixXd::Zero(8, N);
    MatrixXd u_traj = MatrixXd::Zero(2, N);
    double ball_height;

    // Simulate desired human action
    x_traj.col(0) = step_forward(x, u);

    // Main loop
    double t_curr = 0.0;
    int n0 = 0;
    int nf = NN;
    while (t_curr < (T-0.00001))
    {
        // Simulate (x,p)
        for (int n = n0;  n < nf;  n++)
        {
            x_traj.col(n+1) = step_forward(x_traj.col(n),u_traj.col(n));
        }
        
        Vector8d pT = Vector8d::Zero(); // terminal condition
        MatrixXd p_traj = MatrixXd::Zero(8, N);
        p_traj.col(nf) = pT; // not necessary because of previous line
        for (int n = nf; n > n0; n--)
        {
            p_traj.col(n-1) = step_backward(p_traj.col(n), x_traj.col(n), u_traj.col(n));
        }
      
        // Calculate initial cost and specify alpha
        double cost_init = cost(x_traj(all, seq(n0,nf)), u_traj(all, seq(n0, nf)));
        double alpha_d = gamma * cost_init;
        //std::cout << cost_init << std::endl;

        // Compute u2*
        MatrixXd u2 = MatrixXd::Zero(2, N);
        Matrix82d Bmat;
        Matrix2d Lam;
        Vector8d p;
        Vector2d temp;
        for (int n = n0; n < nf; n++) 
        {
            Bmat = Matrix82d::Zero(8, 2); // zero out coefficents
            Bmat(1,0) = -cos(x_traj(0,n));
            Bmat(3,1) = -cos(x_traj(2,n));
            Bmat(5,0) = 1.0;
            Bmat(7,1) = 1.0;
            p = p_traj.col(n);
            Lam = Bmat.transpose() * p * p.transpose() * Bmat;
            temp = (Lam + R.transpose()).inverse() * ((Lam * u_traj.col(n)) + (Bmat.transpose() * p * alpha_d));
            // saturate
            if (temp(0) > u_max) {temp(0) = u_max;}
            else if (temp(0) < u_min) {temp(0) = u_min;}
            if (temp(1) > u_max) {temp(1) = u_max;}
            else if (temp(1) < u_min) {temp(1) = u_min;}
            u2.col(n) = temp;
        }

        // Search for time tau
        VectorXd J_tau(NN);
        VectorXd dJdlam(NN);
        int i = 0, j;
        for (int n = n0; n < nf; n++)
        {
            double delta_t = time(n) - t_curr;
            p = p_traj.col(n);
            dJdlam(i) = p.transpose() * (f(x_traj.col(n), u2.col(n)) - f(x_traj.col(n), u_traj.col(n)));
            J_tau(i) = (u2.col(n)).norm() + dJdlam(i) + pow(delta_t,beta);
            i++;
        }
        J_tau.minCoeff(&j);
        double tau = time(j + n0);

        // Corresponding control 
        Vector2d utau = u2.col(j + n0);
        

        // Set duration (usually the duration is ~searched~ for here)
        double lam = dt;
        u_traj.col(j + n0) = utau; // replace nominal control with optimal at time tau

        // Resimulate
        for (int n = n0; n < nf; n++)
        {
            x_traj.col(n + 1) = step_forward(x_traj.col(n), u_traj.col(n));
        }

        // New cost
        double J1_new = cost(x_traj(all, seq(n0, nf)), u_traj(all, seq(n0, nf)));
        //std::cout << J1_new << std::endl;

        // Next timestep
        t_curr = t_curr + dt;
        n0++;
        nf++;
    }

    // Check safety
    for (int i = 0; i < N; i++)
    {
        //if (abs(x_traj(0,i)) > (PI/2) || abs(x_traj(2, i)) > (PI / 2))
        ball_height = h * (1 - (cos(x_traj(0, i))*cos(x_traj(2, i))));
        if (ball_height > h_max)
        {
            std::cout << "Replacing" << std::endl;
            // Initialize storage variables
            MatrixXd x_opt = MatrixXd::Zero(8, NN+1);
            x_opt.col(0) = x;
            MatrixXd u_opt = MatrixXd::Zero(2, NN+1);
            //u_opt.col(0) = u; //
            // Simulate (x,p)
            for (int m = 0; m < NN; m++)
            {
                x_opt.col(m+1) = step_forward(x_opt.col(m), u_opt.col(m));
            }
            MatrixXd p_opt = MatrixXd::Zero(8, NN+1);
            for (int m = NN; m > 0; m--)
            {
                p_opt.col(m-1) = step_backward(p_opt.col(m), x_opt.col(m), u_opt.col(m));
            }
            // Calculate initial cost and specify alpha
            double alpha_d = gamma * cost(x_opt(all, seq(0, NN)), u_opt(all, seq(0, NN))); 
            // Compute u2*
            MatrixXd u2_opt = MatrixXd::Zero(2, NN);
            Matrix82d Bmat;
            Matrix2d Lam;
            Vector8d p;
            Vector2d temp;
            for (int m = 0; m < NN; m++)
            {
                Bmat = Matrix82d::Zero(8, 2); // zero out coefficents
                Bmat(1, 0) = -cos(x_traj(0, m));
                Bmat(3, 1) = -cos(x_traj(2, m));
                Bmat(5, 0) = 1.0;
                Bmat(7, 1) = 1.0;
                p = p_opt.col(m);
                Lam = Bmat.transpose() * p * p.transpose() * Bmat;
                temp = (Lam + R.transpose()).inverse() * ((Lam * u_opt.col(m)) + (Bmat.transpose() * p * alpha_d));
                // saturate
                if (temp(0) > u_max) { temp(0) = u_max; }
                else if (temp(0) < u_min) { temp(0) = u_min; }
                if (temp(1) > u_max) { temp(1) = u_max; }
                else if (temp(1) < u_min) { temp(1) = u_min; }
                u2_opt.col(m) = temp;
            }
            // Search for time tau
            VectorXd J_tau(NN);
            VectorXd dJdlam(NN);
            int j;
            for (int m = 0; m < NN; m++)
            {
                double delta_t = time(m);
                p = p_opt.col(m);
                dJdlam(m) = p.transpose() * (f(x_opt.col(m), u2_opt.col(m)) - f(x_opt.col(m), u_opt.col(m)));
                J_tau(m) = (u2_opt.col(m)).norm() + dJdlam(m) + pow(delta_t, beta);
            }
            J_tau.minCoeff(&j);
            if (j == 0) 
            {
                u_send = u2_opt.col(0);
            }
            else 
            {
                u_send = u2_opt.col(0);
                //u_send = Vector2d::Zero();
            }
            //std::cout << u_send << std::endl;
            //std::cout << J_tau(0) << std::endl;
            break; 
        }
    }

    return u_send;
}

#endif 
