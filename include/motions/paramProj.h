/**
 *      @file  spiral.h
 *      @brief  Declaration of ParamProj class
 *
 * ... to complete with detailed description
 *
 *     @author  João Moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  13-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2017, João Moura
 *
 * ===============================================================
 */
#ifndef _PARAMPROJ_
#define _PARAMPROJ_

#include <math.h> // for M_PI
#include <iostream> // print to terminal
#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations
#include <ros/ros.h>

#define ParamProjMsgError(msg) (std::string("ParamProj: ") + (msg) + "\nLine: " + std::to_string(__LINE__) + "; Function " + __PRETTY_FUNCTION__ + "; File " + __FILE__)

template <class T>
class ParamProj
{
	public:
		ParamProj();
		~ParamProj();
		
		double t_end; // Max value of parameter t
		double t_i; // current value of parameter t

		/**
		 * @brief Initialize class: copy trajectory class and computes projection matrix
		 * @param traj: planar trajectory class (can be for instance a spiral trajectory) that contains the methods pos and dpos
		**/
		void Init(const T &traj, const Eigen::Matrix<double, 3, 1> &axis1, const Eigen::Matrix<double, 3, 1> &axis2, double KP, double KD, double Cv);

		/**
		 * @brief Projected velocity: This function computes the projection of the end-effector point in the plane where the 2D trajectory was defined, then it computes the closest trajectory point to the projected point usind gradien descent and then computes a velocity vector so to the end-effector follows the trajectory and corrects the error between current position and closest trajectory point. Finally it transforms this velocity to the end-effector frame and and normalizes it.
		 * @param x: position of the end-effector in the global frame
		 * @param rot: rotation matrix from the local end-effector frame to the global frame G_R_T
		 * @return: end-effector x and y velocity
		**/
		Eigen::Matrix<double, 2, 1> projVel(const Eigen::Matrix<double, 3, 1> &x, const Eigen::Matrix<double, 3, 3> &rot);
		/**
		 * @brief Projects end-effector position onto specified plane of parametric trajectory and stores it as reference point in the parametric trajectory (origin of the parametric motion)
		 * @param x0: position of the end-effector in the global frame
		**/
		void projVelFirstTime(const Eigen::Matrix<double, 3, 1> &x0);

	private:
		// Variables
		T traj_; // trajectory class
		Eigen::Matrix<double, 3, 2> invproj_; // Inverse projection matrix
		Eigen::Matrix<double, 2, 3> proj_; // projection matrix
		Eigen::Matrix<double, 2, 1> xProjRef_; // reference point in projection plane
		double t_; // parameter for parametric trajectory
		double KD_, KP_; // gains of the planar position correction
		double Cv_; // maximum tangential velocity	
};


template <class T>
ParamProj<T>::ParamProj()
{
}

template <class T>
ParamProj<T>::~ParamProj()
{
}

template <class T>
void ParamProj<T>::Init(const T &traj, const Eigen::Matrix<double, 3, 1> &axis1, const Eigen::Matrix<double, 3, 1> &axis2, double KP, double KD, double Cv)
{
	// copy class to private variable
	traj_ = traj;
	// copy gains and maximum velocity
	KP_ = KP; KD_ = KD; Cv_ = Cv;
	// copy final value of parameter T
	t_end = traj.T;
	// get inverse projection matrix
	invproj_ << axis1, axis2;
	// Initialize parameter
	t_ = 0.001;
	// Check if axis span a 2 dimensional space
	Eigen::Matrix<double, 2, 2> provMat = (invproj_.transpose())*invproj_;
	Eigen::FullPivLU<Eigen::Matrix<double,2,2>> lu_decomp(provMat);
	if(lu_decomp.isInvertible()){ 
		// get projection matrix
		proj_ = (provMat.inverse())*(invproj_.transpose());
	}
	else{ // axis do not span 2D space
		throw std::runtime_error(ParamProjMsgError("Axis do not span 2D space"));
	}
}

template <class T>
void ParamProj<T>::projVelFirstTime(const Eigen::Matrix<double, 3, 1> &x0)
{
	xProjRef_ = proj_*x0;
}

template <class T>
Eigen::Matrix<double, 2, 1> ParamProj<T>::projVel(const Eigen::Matrix<double, 3, 1> &x, const Eigen::Matrix<double, 3, 3> &rot)
{
	// Initializations
	int count = 0; // number of iteration for gradient descent
	double DC = 1.0; // gradiant of the function to be minimized
	// Gradient descente rate
	double alpha = 0.2;
	Eigen::Matrix<double, 2, 1> xProj = proj_*x - xProjRef_; // project global end-effector position and subtract reference point (origin of parametric trajectory)
	// Loop to find closest point in trajectory to real projected position
	Eigen::Matrix<double, 2, 1> xTraj, dxTraj, errorProj;
	while((std::abs(DC)>0.001) && (count++<50)){
		// get position for given parameter
		xTraj = traj_.pos(t_);
		// get derivative for diven t_
		dxTraj = traj_.dpos(t_);
		// Distance between spiral and real projected end-effector position
		errorProj = xTraj - xProj;
		// derivative of the distance at t
		DC = (errorProj.transpose())*dxTraj;
		// Update of the trajectory parameter t according the gradient
		t_ -= alpha*DC;
		// Make t always positive
		if(t_<0.0) t_ = -t_;
	}
	// Update parameter t global
	t_i = t_;
	// compute planar velocity
	Eigen::Matrix<double, 2, 1> vProj = KD_*dxTraj + KP_*errorProj;
	// tranform it to 3D global spcae
	Eigen::Matrix<double, 3, 1> vGlobal = invproj_*vProj;
	// transform global to local velocity
	Eigen::Matrix<double, 2, 1> vLocal = ((rot.transpose())*vGlobal).head(2);
	// Norm of the plan velocity
	double vLocal_norm = vLocal.norm();
	// Normalization to have constant tangential velocity of C_v
	if(vLocal_norm>0.0) vLocal *= (Cv_/vLocal_norm);
	return vLocal;
}


#endif // _PARAMPROJ_
