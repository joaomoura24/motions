/**
 *      @file  raster.h
 *      @brief  Declaration of Raster class
 *
 * ... to complete with detailed description
 *
 *     @author  João Moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  21-Mar-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2017, João Moura
 *
 * ===============================================================
 */
#ifndef _RASTER_
#define _RASTER_

#include <math.h> // for M_PI
#include <iostream> // print to terminal
#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations


class Raster
{
	public:
		Raster();
		~Raster();
		// Max value of parameter t
		double T;
		// class methods for obtaining current positio and derivative
		/**
		 * @brief Initialize class, copy input variables to private variables
		**/
		void Init(double turnWidth, double turns, double sideLength);
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
		// Paramiters for raster
		double turns_, sideLength_, turnWidth_;
		double H_, V_, Lc_;
		Eigen::Matrix<int,2,1> SIG_;
};

#endif // _RASTER_
