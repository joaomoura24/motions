/**
 *      @file  motions.cpp
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
#include <motions/motions.h>

Motions::Motions()
{
	// get kinematics for toolLink
	std::string toolLink; // End-effector/tool link name
	ros::param::param<std::string>("~toolLink",toolLink,"contact_point");
	// Initialize robot class
	robot_.Init(toolLink);
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
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setJointPos_, this, q), false); // true is for oneshot call function
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
		// get duration time of the movement: DT
		double DT; ros::param::param<double>("~DT", DT, 5.0);
		// set final time of movement
		ros::Time finalT = ros::Time::now() + ros::Duration(DT);
		// get desired joint velocity
		Eigen::Matrix<double, 6, 1> v = readVecFromParam_("~toolVel", 6, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setToolVel_, this, v, finalT), false);
	}
	//-----------------------------------------
	// toolLocVel
	//-----------------------------------------
	else if(controlMode.compare("toolLocVel")==0){
		ROS_ERROR_STREAM("Here");
		// get inverse kinematics weights
		weights_ = readVecFromParam_("~invKinWeights", robot_.kin.nrJoints, 1.0);
		// get duration time of the movement: DT
		double DT; ros::param::param<double>("~DT", DT, 5.0);
		// set final time of movement
		ros::Time finalT = ros::Time::now() + ros::Duration(DT);
		// get desired joint velocity
		Eigen::Matrix<double, 6, 1> v = readVecFromParam_("~toolVel", 6, 0.0);
		// method called with given frequency
		cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), boost::bind(&Motions::setToolLocVel_, this, v, finalT), false);
	}
	//-----------------------------------------
	// contact
	//-----------------------------------------
	else if(controlMode.compare("contact")==0){
		// force considered for detecting contact
		ros::param::param<double>("~Fcontact", Fcontact_, 2.0);
		// Read filtering constant
		double alpha; ros::param::param<double>("~alpha", alpha, 0.9);
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
		Eigen::Matrix<double, 6, 1> fMax;
		fMax = readVecFromParam_("~fMax", 6, 0.0);
		Eigen::Matrix<double, 6, 1> fMin;
		fMin = -fMax;
		// get force sensor link name
		std::string forceLink;
		ros::param::param<std::string>("~forceLink",forceLink,"contact_point");
		// get 6D force transformation matrix
		Eigen::Matrix<double, 3, 1> pos;
		Eigen::Matrix<double, 3, 3> rot;
		robot_.kin.getBetweenFK(pos, rot, robot_.jointValues, forceLink, toolLink);
		Eigen::Matrix<double, 6, 6> TransSensorToContact;
		TransSensorToContact << rot.transpose(), Eigen::Matrix<double, 3, 3>::Zero(), -(rot.transpose())*robot_.kin.makeSkewSymmetric(pos), rot.transpose();
		// Initialize force sensing class
		fsensor_.Init(TransSensorToContact, fMin, fMax, alpha);
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
		// Read filtering constant
		double alpha; ros::param::param<double>("~alpha", alpha, 0.9);
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
		Eigen::Matrix<double, 6, 1> fMax;
		fMax = readVecFromParam_("~fMax", 6, 0.0);
		Eigen::Matrix<double, 6, 1> fMin;
		fMin = -fMax;
		// get force sensor link name
		std::string forceLink;
		ros::param::param<std::string>("~forceLink",forceLink,"contact_point");
		// get 6D force transformation matrix
		Eigen::Matrix<double, 3, 1> pos;
		Eigen::Matrix<double, 3, 3> rot;
		robot_.kin.getBetweenFK(pos, rot, robot_.jointValues, forceLink, toolLink);
		Eigen::Matrix<double, 6, 6> TransSensorToContact;
		TransSensorToContact << rot.transpose(), Eigen::Matrix<double, 3, 3>::Zero(), -(rot.transpose())*robot_.kin.makeSkewSymmetric(pos), rot.transpose();
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
		// Initialize publisher
		bagDataPose_ = nh_.advertise<geometry_msgs::Pose>("/data/pose", 1);
		bagDataForce_ = nh_.advertise<geometry_msgs::WrenchStamped>("/data/force", 1);
		// Initialize force sensing class
		fsensor_.Init(TransSensorToContact, fMin, fMax, alpha);
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
	// Variable for computing error Dq
	Eigen::Matrix<double, Eigen::Dynamic, 1> Dq; Dq.resize(q.size()); Dq.setZero();
	Dq = robot_.jointValues - q;
	if(Dq.norm() < 0.001) ros::shutdown();
	else robot_.setJointPos(q);
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
		else nh_.createTimer(ros::Duration(0.1), boost::bind(&Motions::setJointPos_, this, qIK), false); // true is for oneshot call function
	}
	else{
		// reschedule call of this function
		cycleTimer_.stop();
		cycleTimer_.setPeriod(ros::Duration(1.0)); // some time so the current configurations is read
		cycleTimer_.start();
	}
}

void Motions::setToolVel_(const Eigen::Matrix<double, 6, 1> &v, const ros::Time& finalT)
{
	if(robot_.gotRobotState){
		if(ros::Time::now() > finalT) ros::shutdown();
		else{
			// get psedo inverse Jacobian
			Eigen::Matrix<double, Eigen::Dynamic, 6> jacobianInv;
			robot_.kin.getJacInv(jacobianInv, robot_.jointValues, weights_);
			// get joint velocities
			Eigen::Matrix<double, Eigen::Dynamic, 1> dq;
			dq = jacobianInv*v;
			// set joint velocities
			robot_.setJointVel(dq);
		}
	}
	else ROS_WARN_STREAM("Motions: Wainting for robot state");
}

void Motions::setToolLocVel_(const Eigen::Matrix<double, 6, 1> &v, const ros::Time& finalT)
{
	if(robot_.gotRobotState){
		if(ros::Time::now() > finalT) ros::shutdown();
		else{
			// get psedo inverse Jacobian
			Eigen::Matrix<double, Eigen::Dynamic, 6> jacobianInv;
			robot_.kin.getLocJacInv(jacobianInv, robot_.jointValues, weights_);
			// get joint velocities
			Eigen::Matrix<double, Eigen::Dynamic, 1> dq;
			dq = jacobianInv*v;
			// set joint velocities
			robot_.setJointVel(dq);
		}
	}
	else ROS_WARN_STREAM("Motions: Wainting for robot state");
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
	if(fsensor_.gotForceSensing && robot_.gotRobotState){
		Eigen::Matrix<double, 6, 1> vel;
		if((fsensor_.force.cwiseAbs()).maxCoeff() < Fcontact_) vel.setZero(); // if there is no contact stay still
		else{
			// Compute velocity to minimize difference from force goal and minimize derivative term
			vel = -Kcontact_.cwiseProduct(fGoal_ - fsensor_.force) - dKcontact_.cwiseProduct(fsensor_.forceDerivative);
			// Saturate output to maximum velocity
			for(int idx=0; idx<vel.size(); idx++) vel(idx) = VAL_SAT(vel(idx), -vMaxContact_(idx), vMaxContact_(idx));
		}
		ROS_INFO_STREAM(vel.transpose());
		setToolLocVel_(vel, ros::Time::now() + ros::Duration(10));
	}
}

void Motions::sweeping_()
{
	if(fsensor_.gotForceSensing && robot_.gotRobotState){
		static bool firstTime = true;
		Eigen::Matrix<double, 6, 1> vel;
		if((fsensor_.force.cwiseAbs()).maxCoeff() < Fsweep_) vel << 0.0, 0.0, velZ_, 0.0, 0.0, 0.0; // if there is no contact move
		else{
			// Compute velocity to minimize difference from force goal and minimize derivative term
			vel = -Ksweep_.cwiseProduct(fGoal_ - fsensor_.force) - dKsweep_.cwiseProduct(fsensor_.forceDerivative);
			// get forward kinematics
			Eigen::Matrix<double, 3, 1> pos;
			Eigen::Matrix<double, 3, 3> rot;
			robot_.kin.getFK(pos, rot, robot_.jointValues);
			// --------------------------------------------------------------------------------
			// Publish pose
			Eigen::Matrix<double, 3, 1> pos2;
			Eigen::Matrix<double, 4, 1> quat;
			robot_.kin.getFK(pos2, quat, robot_.jointValues);
			geometry_msgs::Pose pose; // create msg
			pose.position.x = pos2[0];
			pose.position.y = pos2[1];
			pose.position.z = pos2[2];
			pose.orientation.x = quat[0];
			pose.orientation.y = quat[1];
			pose.orientation.z = quat[2];
			pose.orientation.w = quat[3];
			bagDataPose_.publish(pose);
			// Publish force
			geometry_msgs::WrenchStamped wrench_msg; // create msg
			//wrench_msg.header.stamp = msg->header.stamp;
			wrench_msg.wrench.force.x = fsensor_.force[0];
			wrench_msg.wrench.force.y = fsensor_.force[1];
			wrench_msg.wrench.force.z = fsensor_.force[2];
			wrench_msg.wrench.torque.x = fsensor_.force[3];
			wrench_msg.wrench.torque.y = fsensor_.force[4];
			wrench_msg.wrench.torque.z = fsensor_.force[5];
			bagDataForce_.publish(wrench_msg);
			// --------------------------------------------------------------------------------
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
		setToolLocVel_(vel, ros::Time::now() + ros::Duration(10));
	}
}

int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node, registering with the master as SweepingRobot
	ros::init(argc, argv, "Motions", ros::init_options::AnonymousName);
	ros::NodeHandle dummy_handle; // because of clash between throw exception and ROS handle
	try{
		// Initializing robot class (it has to be after init_node)
		Motions motion;
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
