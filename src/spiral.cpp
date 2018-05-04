/**
 *      @file  spiral.cpp
 *      @brief  Definition of Spiral class
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  13-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2016, João Moura
 *
 * ===============================================================
 */

#include <motions/spiral.h>

Spiral::Spiral()
{
}

void Spiral::Init(double radius, double Nturns, double phase)
{
	// Copy spiral parameters:
	radius_ = radius;
	turns_ = Nturns;
	phase_ = phase;
	// Max value of parameter:
	T = 2*M_PI*turns_;
}

Spiral::~Spiral()
{
}

Eigen::Matrix<double,2,1> Spiral::pos(double t)
{
	// Avoid negative parameter
	if(t<0.0) t = -t;
	// Spiral radious:
	double r = (t/(turns_*2*M_PI))*radius_;
	// Spiral angle
	double theta = t + phase_;
	// Archimedean Spiral
	Eigen::Matrix<double,2,1> p;
	p << r*cos(theta), r*sin(theta);
	return p;
}

Eigen::Matrix<double,2,1> Spiral::dpos(double t)
{
	// get current point in trajectory
	Eigen::Matrix<double,2,1> x_traj = pos(t);
	// Avoid negative parameter
	if(t<0.0) t = -t;
	Eigen::Matrix<double,2,2> d_rot;
	d_rot << 1.0/t, -1.0, 1.0, 1.0/t;
	Eigen::Matrix<double,2,1> dx;
	dx = d_rot*x_traj;
	double dx_n = dx.norm();
	if(dx_n>0.0) dx /= dx_n;
	return dx;
}
