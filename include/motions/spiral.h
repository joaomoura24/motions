/**
 *      @file  spiral.h
 *      @brief  Declaration of Spiral class
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
#ifndef _SPIRAL_
#define _SPIRAL_

#include <math.h> // for M_PI
#include <iostream> // print to terminal
#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations


class Spiral
{
	public:
		Spiral();
		~Spiral();
		// Max value of parameter t
		double T;
		// class methods for obtaining current positio and derivative
		/**
		 * @brief Initialize class, copy input variables to private variables
		 * @param radius: outer radius of spiral
		 * @param Nturns: number of turns of spiral
		 * @param phase: initial angle of spiral
		**/
		void Init(double radius, double Nturns, double phase);
		/**
		 * @brief Method that returns trajectory position for parameter t
		 * @param t: parameter
		 * @return 2d position in trajectory for given parameter t
		**/
		Eigen::Matrix<double,2,1> pos(double t);
		/**
		 * @brief Method that returns trajectory derivative (direction) for parameter t
		 * @param t: parameter
		 * @return 2d vector trajectory gradient at parameter t (normalized vector)
		**/
		Eigen::Matrix<double,2,1> dpos(double t);
	private:
		// Paramiters for spiral
		double radius_;
		double turns_;
		double phase_;
};

#endif // _SPIRAL_
