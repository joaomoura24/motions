/**
 *      @file  motions_kuka.cpp
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
#include <motions/motions_kuka.h>

Motions::Motions()
{
	gotForceSensing_ = false; // did not get force readings
	// get kinematics for toolLink
	std::string toolLink; // End-effector/tool link name
	ros::param::param<std::string>("~toolLink",toolLink,"contact_point");
	// Initialize robot class
	robot_.init(toolLink);
	// get cycle frequency
	double freq; ros::param::param<double>("~freq", freq, 50.0);
	// get operation mode:
	std::string controlMode; ros::param::param<std::string>("~controlMode",controlMode,"nonsense");
	//-----------------------------------------
	// jointPos
	//-----------------------------------------
	if(controlMode.compare("jointPos")==0){
		// get desired joint positions
		Eigen::Matrix<double, Eigen::Dynamic, 1> q = readVecFromParam_("~jointPos", robot_.kin.nrJoints, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setJointPos_, this, q), true); // true is for oneshot call function
	}
	//-----------------------------------------
	// jointVel
	//-----------------------------------------
	else if(controlMode.compare("jointVel")==0){
		// get desired joint velocity
		Eigen::Matrix<double, Eigen::Dynamic, 1> dq = readVecFromParam_("~jointVel", robot_.kin.nrJoints, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setJointVel_, this, dq), false);
	}
	//-----------------------------------------
	// toolPos
	//-----------------------------------------
	else if(controlMode.compare("toolPos")==0){
		// get desired tool position
		Eigen::Matrix<double, 3, 1> pos = readVecFromParam_("~toolPos", 3, 0.0);
		// get axis of rotation for the tool orientation
		Eigen::Matrix<double, 3, 1> axis = readVecFromParam_("~toolRotAxis", 3, 1.0);
		// normalize axis vector
		if(axis.norm()>0.0) axis.normalize();
		else throw std::runtime_error("Rotation Axis norm is not positive");
		// get rotation angle about the axis of rotation
		double angle; ros::param::param<double>("~toolRotAngle", angle, 0.0);
		// convert axis and angle to quaternion
		double sin_val = sin(angle*M_PI/360);
		double cos_val = cos(angle*M_PI/360);
		Eigen::Matrix<double, 4, 1> quat;
		quat << axis[0]*sin_val, axis[1]*sin_val, axis[2]*sin_val, cos_val;
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setToolPos_, this, pos, quat), true); // true is for oneshot call function
	}
	//-----------------------------------------
	// toolVel
	//-----------------------------------------
	else if(controlMode.compare("toolVel")==0){
		// get inverse kinematics weights
		weights_ = readVecFromParam_("~invKinWeights", robot_.kin.nrJoints, 1.0);
		// get desired joint velocity
		Eigen::Matrix<double, 6, 1> v = readVecFromParam_("~toolVel", 6, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setToolVel_, this, v), false);
	}
	//-----------------------------------------
	// toolLocVel
	//-----------------------------------------
	else if(controlMode.compare("toolLocVel")==0){
		// get inverse kinematics weights
		weights_ = readVecFromParam_("~invKinWeights", robot_.kin.nrJoints, 1.0);
		// get desired joint velocity
		Eigen::Matrix<double, 6, 1> v = readVecFromParam_("~toolVel", 6, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setToolLocVel_, this, v), false);
	}
	//-----------------------------------------
	// contact
	//-----------------------------------------
	else if(controlMode.compare("contact")==0){
		// force considered for detecting contact
		ros::param::param<double>("~Fcontact", Fcontact_, 2.0);
		// Initialize force variables
		f_.setZero(); df_.setZero(); fFilter_.setZero();
		// Read filtering constant
		ros::param::param<double>("~alpha", alpha_, 0.9);
		// get inverse kinematics weights
		fGoal_ = readVecFromParam_("~forceGoal", 6, 0.0);
		// get proportional gains
		Kcontact_ = readVecFromParam_("~Kcontact", 6, 0.0);
		// get derivative gains
		dKcontact_ = readVecFromParam_("~dKcontact", 6, 0.0);
		// get end-effector velocity limits
		vMaxContact_ = readVecFromParam_("~vMaxContact", 6, 0.0);
		// get inverse kinematics weights
		weights_ = readVecFromParam_("~invKinWeights", robot_.kin.nrJoints, 1.0);
		// get maximum end-effector force
		fMax_ = readVecFromParam_("~fMax", 6, 0.0);
		fMin_ = -fMax_;
		// get force sensor link name
		std::string forceLink;
		ros::param::param<std::string>("~forceLink",forceLink,"contact_point");
		// get 6D force transformation matrix
		Eigen::Matrix<double, 3, 1> pos;
		Eigen::Matrix<double, 3, 3> rot;
		robot_.kin.getBetweenFK(pos, rot, robot_.jointValues, forceLink, toolLink);
		TransSensorToContact_ << rot.transpose(), Eigen::Matrix<double, 3, 3>::Zero(), -(rot.transpose())*robot_.kin.makeSkewSymmetric(pos), rot.transpose();
		// Subscribe to force sensor data
		subForceSensing_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &Motions::callbackForceSensing_, this);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::keepContact_, this), false);
	}
	//-----------------------------------------
	// sweep
	//-----------------------------------------
	else if(controlMode.compare("sweep")==0){
		// velocity for approaching surface
		ros::param::param<double>("~velZ", velZ_, 0.01);
		// force considered for detecting contact
		ros::param::param<double>("~Fsweep", Fsweep_, 2.0);
		// Initialize force variables
		f_.setZero(); df_.setZero(); fFilter_.setZero();
		// Read filtering constant
		ros::param::param<double>("~alpha", alpha_, 0.9);
		// get inverse kinematics weights
		fGoal_ = readVecFromParam_("~forceGoal", 6, 0.0);
		// get proportional gains
		Ksweep_ = readVecFromParam_("~Ksweep", 6, 0.0);
		// get derivative gains
		dKsweep_ = readVecFromParam_("~dKsweep", 6, 0.0);
		// get end-effector velocity limits
		vMaxSweep_ = readVecFromParam_("~vMaxSweep", 6, 0.0);
		// get inverse kinematics weights
		weights_ = readVecFromParam_("~invKinWeights", robot_.kin.nrJoints, 1.0);
		// get maximum end-effector force
		fMax_ = readVecFromParam_("~fMax", 6, 0.0);
		fMin_ = -fMax_;
		// get force sensor link name
		std::string forceLink;
		ros::param::param<std::string>("~forceLink",forceLink,"contact_point");
		// get 6D force transformation matrix
		Eigen::Matrix<double, 3, 1> pos;
		Eigen::Matrix<double, 3, 3> rot;
		robot_.kin.getBetweenFK(pos, rot, robot_.jointValues, forceLink, toolLink);
		TransSensorToContact_ << rot.transpose(), Eigen::Matrix<double, 3, 3>::Zero(), -(rot.transpose())*robot_.kin.makeSkewSymmetric(pos), rot.transpose();
		// get type of trajectory
		double radius; ros::param::param<double>("~radius", radius, 0.1);
		double Nturns; ros::param::param<double>("~Nturns", Nturns, 3.75);
		double phase; ros::param::param<double>("~phase", phase, 0.0);
		PATH_CLASS traj; traj.Init(radius, Nturns, phase);
		// get axis of plane where desired trajectory is defined
		Eigen::Matrix<double, 3, 1> axis1 = readVecFromParam_("~axis1", 3, 0.0);
		Eigen::Matrix<double, 3, 1> axis2 = readVecFromParam_("~axis2", 3, 0.0);
		// get class to compute end-effector velocity based on trajectory projection
		double KP; ros::param::param<double>("~KP", KP, 1.0);
		double KD; ros::param::param<double>("~KD", KD, 100.0);
		double Cv; ros::param::param<double>("~Cv", Cv, 0.02);
		proj_.Init(traj, axis1, axis2, KP, KD, Cv);
		// Subscribe to force sensor data
		subForceSensing_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &Motions::callbackForceSensing_, this);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::sweeping_, this), false);
	}
	//-----------------------------------------
	// Incorrect mode
	//-----------------------------------------
	else throw std::runtime_error(MotionsMsgError("Control mode specified not defined"));
}

Motions::~Motions()
{
}

void Motions::setJointPos_(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	robot_.setJointPos(q);
}

void Motions::setJointVel_(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{
	robot_.setJointVel(dq);
}

void Motions::setToolPos_(const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 4, 1> &quat)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> qIK;
	if(robot_.gotRobotState){
		if(!robot_.kin.getIK(qIK, robot_.jointValues, pos, quat)) ROS_WARN_STREAM("Motions: No IK solution for the tool pose specified");
		else robot_.setJointPos(qIK);
	}
	else{
		// reschedule call of this function
		cycleTimer_.stop();
		cycleTimer_.setPeriod(ros::Duration(1.0)); // some time so the current configurations is read
		cycleTimer_.start();
	}
}

void Motions::setToolVel_(const Eigen::Matrix<double, 6, 1> &v)
{
	if(robot_.gotRobotState){
		// get psedo inverse Jacobian
		Eigen::Matrix<double, Eigen::Dynamic, 6> jacobianInv;
		robot_.kin.getJacInv(jacobianInv, robot_.jointValues, weights_);
		// get joint velocities
		Eigen::Matrix<double, Eigen::Dynamic, 1> dq;
		dq = jacobianInv*v;
		// set joint velocities
		robot_.setJointVel(dq);
	}
	else ROS_WARN_STREAM("Motions: Wainting for robot state");
}

void Motions::setToolLocVel_(const Eigen::Matrix<double, 6, 1> &v)
{
	if(robot_.gotRobotState){
		// get psedo inverse Jacobian
		Eigen::Matrix<double, Eigen::Dynamic, 6> jacobianInv;
		robot_.kin.getLocJacInv(jacobianInv, robot_.jointValues, weights_);
		// get joint velocities
		Eigen::Matrix<double, Eigen::Dynamic, 1> dq;
		dq = jacobianInv*v;
		// set joint velocities
		robot_.setJointVel(dq);
	}
	else ROS_WARN_STREAM("Motions: Wainting for robot state");
}

void Motions::callbackForceSensing_(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	// Storing sensor msg in variable
	f_[0] = msg->wrench.force.x;
	f_[1] = msg->wrench.force.y;
	f_[2] = msg->wrench.force.z;
	f_[3] = msg->wrench.torque.x;
	f_[4] = msg->wrench.torque.y;
	f_[5] = msg->wrench.torque.z;
	// Initial calibration process
	static int count = 0;
	static Eigen::Matrix<double, 6, 1> fcal = Eigen::Matrix<double, 6, 1>::Zero();
	if(!gotForceSensing_){
		fcal += (1.0/++count) * (f_ - fcal); // average first values
		if(count == 1) ROS_INFO_STREAM("Motions: Calibrating force sensor...");
		if(count == 200){
			ROS_INFO_STREAM("Motions: Calibration values:");
			ROS_INFO_STREAM(fcal.transpose());
			gotForceSensing_ = true; // callibration done
		}
	}
	else{
		// Discount initial reading of force sensor
		f_ -= fcal;
		// Transform 6D force from sensor to contact point
		f_ = TransSensorToContact_*f_;
		// filtering force
		static Eigen::Matrix<double, 6, 1> fPrev = Eigen::Matrix<double, 6, 1>::Zero();
		fPrev = fFilter_;
		fFilter_ += alpha_*(f_ - fFilter_);
		// computing the derivative
		df_ = fFilter_ - fPrev;
	}
    //-----------------------------------------------------------------
    // SAFETY CONDITIONS
    //-----------------------------------------------------------------
    // End-effector force (force sensor)
    maxLimError_("END-EFFECTOR FORCE", f_, fMin_, fMax_);
    //-----------------------------------------------------------------
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Motions::readVecFromParam_(std::string paramName, int vecSize, double defaultValue)
{
	std::vector<double> vect(vecSize);
	std::vector<double> vect0(vecSize, defaultValue);
	ros::param::param<std::vector<double>>(paramName, vect, vect0);
	return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>((vect).data(), (vect).size());
}

void Motions::keepContact_()
{
	if(gotForceSensing_ && robot_.gotRobotState){
		Eigen::Matrix<double, 6, 1> vel;
		if((f_.cwiseAbs()).maxCoeff() < Fcontact_) vel.setZero(); // if there is no contact stay still
		else{
			// Compute velocity to minimize difference from force goal and minimize derivative term
			vel = -Kcontact_.cwiseProduct(fGoal_ - f_) - dKcontact_.cwiseProduct(df_);
			// Saturate output to maximum velocity
			for(int idx=0; idx<vel.size(); idx++) vel(idx) = VAL_SAT(vel(idx), -vMaxContact_(idx), vMaxContact_(idx));
		}
		setToolLocVel_(vel);
	}
}

void Motions::sweeping_()
{
	if(gotForceSensing_ && robot_.gotRobotState){
		static bool firstTime = true;
		Eigen::Matrix<double, 6, 1> vel;
		if((f_.cwiseAbs()).maxCoeff() < Fsweep_) vel << 0.0, 0.0, velZ_, 0.0, 0.0, 0.0; // if there is no contact move
		else{
			// Compute velocity to minimize difference from force goal and minimize derivative term
			vel = -Ksweep_.cwiseProduct(fGoal_ - f_) - dKsweep_.cwiseProduct(df_);
			// get forward kinematics
			Eigen::Matrix<double, 3, 1> pos;
			Eigen::Matrix<double, 3, 3> rot;
			robot_.kin.getFK(pos, rot, robot_.jointValues);
			if(firstTime){
				proj_.projVelFirstTime(pos);
				firstTime = false;
			}
			else{
				// get end-effector velocity
				Eigen::Matrix<double, 2, 1> velParam = proj_.projVel(pos, rot);
				// replace x and y velocity by velocity from parametric trajectory following
				vel.head(2) = velParam;
			}
			// Saturate output to maximum velocity
			for(int idx=0; idx<vel.size(); idx++) vel(idx) = VAL_SAT(vel(idx), -vMaxSweep_(idx), vMaxSweep_(idx));
		}
		ROS_INFO_STREAM(vel.transpose());
		setToolLocVel_(vel);
	}
}

template <typename Derived>
void Motions::maxLimError_(const std::string name, const Eigen::DenseBase<Derived>& var, const Eigen::DenseBase<Derived>& var_min, const Eigen::DenseBase<Derived>& var_max)
{
    for(int idx=0; idx<var.size(); idx++){
        if(var[idx] < var_min[idx] || var[idx] > var_max[idx]){
            ROS_ERROR_STREAM("SweepingRobot: LIMIT " << name << " REACHED");
            ROS_ERROR_STREAM("MIN " << name << ": " << var_min.transpose());
            ROS_ERROR_STREAM(name << ": " << var.transpose());
            ROS_ERROR_STREAM("MAX " << name << ": " << var_max.transpose());
            ros::shutdown();
        }
    }
}

int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node, registering with the master as SweepingRobot
	ros::init(argc, argv, "MotionsKuka", ros::init_options::AnonymousName);
	ros::NodeHandle dummy_handle; // because of clash between throw exception and ROS handle
	try{
		// Initializing robot class (it has to be after init_node)
		Motions kuka;
		// Let ROS take over.
		ros::spin();
	}
	//catch(int msg){
	catch (std::exception &e) {
		//ROS_ERROR_STREAM(msg);
		ROS_ERROR_STREAM(e.what());
		ROS_ERROR_STREAM("Motions: Terminating node!");
		return 1;
	}
	// Cleaning and returning
	return 0;
}
