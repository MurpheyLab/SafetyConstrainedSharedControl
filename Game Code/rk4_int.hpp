//---------------------------------------------------------------------
// rk4_int.hpp
// This is a Runge-Kutte integration scheme that takes a class with 
// dynamics f and evolves it forward in time.
// Backwards integration can be performed by passing a negative value
// as the argument for dt.
//---------------------------------------------------------------------

#ifndef RK4_INT_HPP
#define RK4_INT_HPP
//#include<vector>

#include<Eigen>
using namespace Eigen;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 2, 1> Vector2d;

template <class T> Vector8d RK4_step(T* sys, const Vector8d& x, Vector2d& u, double dt) {
	Vector8d k1, k2, k3, k4;
	k1 = dt * sys->f(x, u);
	k2 = dt * sys->f(x + (0.5 * k1), u);
	k3 = dt * sys->f(x + (0.5 * k2), u);
	k4 = dt * sys->f(x + k3, u);
	return x + ((1. / 6.) * k1) + ((1. / 3.) * k2) + ((1. / 3.) * k3) + ((1. / 6.) * k4);
};

template <class T> Vector8d RK4_back(T* sys, const Vector8d& p, const Vector8d& x, Vector2d& u, double dt) {
	Vector8d k1, k2, k3, k4;
	k1 = dt * sys->pdot(p, x, u);
	k2 = dt * sys->pdot(p + (0.5 * k1), x, u);
	k3 = dt * sys->pdot(p + (0.5 * k2), x, u);
	k4 = dt * sys->pdot(p + k3, x, u);
	return p + ((1. / 6.) * k1) + ((1. / 3.) * k2) + ((1. / 3.) * k3) + ((1. / 6.) * k4);
};

#endif