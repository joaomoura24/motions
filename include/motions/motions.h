/**
 *      @file  motions.h
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
// Motion Class
#include <motions/spiral.h>
#include <motions/raster.h>
// projection class
#include <motions/paramProj.h>
// robot header
#include <hardware/kuka.h>
#include <hardware/cab.h>
// sensor header
#include <hardware/optoforce.h>
#include <hardware/gamma.h>
// include ros message for end-effector pose
#include <geometry_msgs/Pose.h>

#define MotionsMsgError(msg) (std::string("Motions: ") + (msg) + "\nLine: " + std::to_string(__LINE__) + "; Function " + __PRETTY_FUNCTION__ + "; File " + __FILE__)

// ----------------------------------------------
// Define classes being used
// ----------------------------------------------
// Raster or Spiral
#define PATH_CLASS Raster
// Cab or Kuka
#define ROBOT Cab
// OptoForce or Gamma
#define FORCESENSOR OptoForce

class Motions
{
	public:
		Motions(); // class constructor
		virtual ~Motions(); // destructor
	private:
		//-----------------------------------------
		// Initialize Robot and Force sensor classes
		//-----------------------------------------
		ROBOT robot_;
		FORCESENSOR fsensor_;
		//-----------------------------------------
		// Parametric trajectory related variables
		//-----------------------------------------
		ParamProj<PATH_CLASS> proj_;
		//-----------------------------------------
		// Kinematic related variables
		//-----------------------------------------
		Eigen::Matrix<double, Eigen::Dynamic, 1> weights_; // weights for inverse kinematics
		bool gotForceSensing_;
		//-----------------------------------------
		// Forse sensor related variables
		//-----------------------------------------
		Eigen::Matrix<double, 6, 1> fGoal_;
		Eigen::Matrix<double, 6, 1> Kcontact_;
		Eigen::Matrix<double, 6, 1> dKcontact_;
		Eigen::Matrix<double, 6, 1> vMaxContact_;
		Eigen::Matrix<double, 6, 1> Ksweep_;
		Eigen::Matrix<double, 6, 1> dKsweep_;
		Eigen::Matrix<double, 6, 1> vMaxSweep_;
		double Fcontact_, Fsweep_, velZ_;
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
		void setToolVel_(const Eigen::Matrix<double, 6, 1> &v, const ros::Time &finalT);
		/**¬
		 * @brief Commands the robot arm tool velocity in the local frame
		 * @param v: tool velocity (6 dimensional vector) in the global frame
		 */
		void setToolLocVel_(const Eigen::Matrix<double, 6, 1> &v, const ros::Time& finalT);
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
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::Timer cycleTimer_;
		ros::NodeHandle nh_; // ROS node handle
		// publisher variable
		ros::Publisher bagDataPose_;
		ros::Publisher bagDataForce_;
};
