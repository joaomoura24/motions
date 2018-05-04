/**
 *      @file  raster.cpp
 *      @brief  Definition of Raster class
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

#include <motions/raster.h>

Raster::Raster()
{
}

void Raster::Init(double turnWidth, double turns, double sideLength)
{
	// Copy raster parameters:
	turns_ = turns;
	sideLength_ = sideLength;
	turnWidth_ = turnWidth;
	H_ = 0.5*sideLength_; // Horizontal lenght
	V_ = turnWidth_; // vertical lenght
	Lc_ = sideLength_ + (0.5*M_PI-1.0)*V_; // Length of a cycle
	// Max value of parameter:
	T = turns_*Lc_;
	// inidicates positive and negative velocities
    SIG_ << -1, 1;
}

Raster::~Raster()
{
}

Eigen::Matrix<double,2,1> Raster::pos(double t)
{
	// Avoid negative parameter
	if(t<0.0) t = -t;
	// Number of cycles
	int N_c = t/Lc_;
	// Remaining length
	double L_left = t - N_c*Lc_;
	// sign of trajectory
	int sig = SIG_[N_c%2];
	// Choose line of trajectory
	Eigen::Matrix<double,2,1> x;
	if(L_left < (H_-0.5*V_)) x << N_c*V_, sig*L_left;
	else if(L_left < (H_ + 0.5*((M_PI)-1.0)*V_)) x << (N_c+0.5)*V_ + 0.5*(V_)*sin(sig*(L_left-H_+0.5*V_)*2.0/(V_)-0.5*M_PI), sig*(H_-0.5*V_) + 0.5*(V_)*cos(sig*(L_left-H_+0.5*V_)*2.0/(V_)-0.5*M_PI);
	else x << (N_c + 1)*V_, sig*(2.0*H_ + ((0.5*M_PI)-1.0)*V_-L_left);
	return x;
}

Eigen::Matrix<double,2,1> Raster::dpos(double t)
{
	// Avoid negative parameter
	if(t<0.0) t = -t;
	// Number of cycles
	int N_c = t/Lc_;
	// Remaining length
	double L_left = t - N_c*Lc_;
	// sign of trajectory
	int sig = SIG_[N_c%2];
	// Choose line of trajectory
	Eigen::Matrix<double,2,1> dx;
	if(L_left < (H_-0.5*V_)) dx << 0.0, sig;
	else if(L_left < (H_ + 0.5*((M_PI)-1.0)*V_)) dx << sig*cos(sig*(L_left-H_+0.5*V_)*2.0/(V_)-0.5*M_PI), -sig*sin(sig*(L_left-H_+0.5*V_)*2.0/(V_)-0.5*M_PI);
	else dx << 0.0, -sig;
	return dx;
}
