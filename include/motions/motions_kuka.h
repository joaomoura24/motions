/**
 *      @file  motions_kuka.h
 *      @brief  Implementation of motion primitives such as jointpos, jointVel, toolPos, and toolVel for the Kuka lightweight arm
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  09-Oct-2016
 *     Created  18-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 * ===============================================================
 */

#include <math.h> // for sin cos
// print to terminal
#include <iostream>
// standard string
#include <string>
// ROS header
#include <ros/ros.h>
// Spiral Class
#include <parametric_motions/spiral.h>
// projection class
#include <parametric_motions/paramProj.h>
// robot header
#include <robots/kuka.h>

#define MotionsMsgError(msg) (std::string("Motions: ") + (msg) + "\nLine: " + std::to_string(__LINE__) + "; Function " + __PRETTY_FUNCTION__ + "; File " + __FILE__)

#define PATH_CLASS Spiral
#define ROBOT Kuka

class Motions
{
	public:
		Motions(); // class constructor
		virtual ~Motions(); // destructor
	private:
		//-----------------------------------------
		// Initialize Robot class
		//-----------------------------------------
		ROBOT robot_; // (toolLink);
		//-----------------------------------------
		// Kinematic related variables
		//-----------------------------------------
		Eigen::Matrix<double, Eigen::Dynamic, 1> weights_; // weights for inverse kinematics
		bool gotForceSensing_;
		//-----------------------------------------
		// Forse sensor related variables
		//-----------------------------------------
		Eigen::Matrix<double, 6, 1> f_; // force measured after calibration
		Eigen::Matrix< double, 6, 1> fFilter_; // filtered force
		Eigen::Matrix< double, 6, 1> df_; // derivative of filtered force
		double alpha_; // filtering constant
		Eigen::Matrix<double, 6, 6> TransSensorToContact_;
		Eigen::Matrix<double, 6, 1> fGoal_;
		Eigen::Matrix<double, 6, 1> Kcontact_;
		Eigen::Matrix<double, 6, 1> dKcontact_;
		Eigen::Matrix<double, 6, 1> vMaxContact_;
		Eigen::Matrix<double, 6, 1> Ksweep_;
		Eigen::Matrix<double, 6, 1> dKsweep_;
		Eigen::Matrix<double, 6, 1> vMaxSweep_;
		Eigen::Matrix<double, 6, 1> fMax_;
		Eigen::Matrix<double, 6, 1> fMin_;
		double Fcontact_, Fsweep_, velZ_;
		//-----------------------------------------
		// Parametric trajectory related variables
		//-----------------------------------------
		ParamProj<PATH_CLASS> proj_;
		//-----------------------------------------
		// Private methods
		//-----------------------------------------
		/**¬
		 * @brief Commands the robot arm joint positions
		 * @param q: joint positions
		 */
		void setJointPos_(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);
		/**¬
		 * @brief Commands the robot arm joint velocities
		 * @param q: joint velocities
		 */
		void setJointVel_(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
		/**¬
		 * @brief Commands the robot arm tool position and orientation
		 * @param pos: tool position
		 * @param quat: tool orientation quaternion [qx, qy, qz, qw]
		 */
		void setToolPos_(const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 4, 1> &quat);
		/**¬
		 * @brief Commands the robot arm tool velocity in the global frame
		 * @param v: tool velocity (6 dimensional vector) in the global frame
		 */
		void setToolVel_(const Eigen::Matrix<double, 6, 1> &v);
		/**¬
		 * @brief Commands the robot arm tool velocity in the local frame
		 * @param v: tool velocity (6 dimensional vector) in the global frame
		 */
		void setToolLocVel_(const Eigen::Matrix<double, 6, 1> &v);
		/**¬
		 * @brief Reads force sensor values and calibrates sensor
		 * @param msg: force sensor message
		 */
		void callbackForceSensing_(const geometry_msgs::WrenchStamped::ConstPtr& msg);
		/**¬
		 * @brief Reads joints position and stores it in private vector
		 * @param msg: robot state message
		 */
		Eigen::Matrix<double, Eigen::Dynamic, 1> readVecFromParam_(std::string paramName, int vecSize, double defaultValue);
		/**¬
		 * @brief adjust end-effector orientation and pressure to the surface
		 */
		void keepContact_();
		/**¬
		 * @brief sweeping according specified type of motion while adjusting end-effector orientation and pressure on surface
		 */
		void sweeping_();
		/**
		 * @brief function to check limit of a variable and terminate program in case the limit is reached
		 * @param name: error message
		 * @param var: vector to control
		 * @param var_min: minimum values
		 * @param var_max: maximum values
		 */
		template <typename Derived>
		void maxLimError_(const std::string name, const Eigen::DenseBase<Derived>& var, const Eigen::DenseBase<Derived>& var_min, const Eigen::DenseBase<Derived>& var_max);
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::Timer cycleTimer_;
		ros::Subscriber subForceSensing_;
		ros::NodeHandle nh_; // ROS node handle
		//-----------------------------------------
		// Variables specific to robot:
		//-----------------------------------------
};
