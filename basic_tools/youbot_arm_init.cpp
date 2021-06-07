/* ============================ Choose the case in the commandline ================================== */
/*                                                                                                    */
/*   Usage ?                                                                                          */
/*      > rosrun <rospkg name> <name of the executable file> <mode>                                   */
/*      > Example : rosrun arm_control_test youbot_arm_init home                                      */
/*                                                                                                    */
/*   How to add a new case ?                                                                          */
/*      > 1. add a new case in std::map [ mode ] ( from line 36 )                                     */
/*      > 2. add a new set of desired joints in function [ getPostions ]                              */
/*      > 3. add a new set of desired rotation sequence of joints in function [ jointSeq ]            */
/*                                                                                                    */
/* ================================================================================================== */

#include <iostream>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

#define DEG2RAD 0.01745329

map<string, int>mode = {
	{"home", 1},
	{"find_line", 2},
};

const int NumOfJoint = 5;
const string arm_joint_name[NumOfJoint] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};

ros::Publisher armPositionsPublisher;

// =========================================/

brics_actuator::JointPositions genCommand(const double joint_value[NumOfJoint], vector<int> joint_num){
	int cmd_num =  joint_num.size();

	brics_actuator::JointPositions command;
	vector<brics_actuator::JointValue> armJointPositions;
	armJointPositions.resize(cmd_num);

	cout << endl;
	
	for(int i = 0; i < cmd_num; i++){
		armJointPositions[i].joint_uri = arm_joint_name[joint_num[i]-1];
		armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
		armJointPositions[i].value = joint_value[joint_num[i]-1];
		cout << "setting: " << armJointPositions[i].joint_uri << ", " << armJointPositions[i].value << endl;
	}
	command.positions = armJointPositions;
	return command;
}

brics_actuator::JointPositions getPostions(const int jointSet, vector<int> jointNum){
	double arm_joint_value[NumOfJoint];
	cout << "--------------------------------------" << endl;
	if (jointSet == mode["home"]){
		cout << "generate position set : home ..." << endl;
		arm_joint_value[0] = 0.0100692;
		arm_joint_value[1] = 0.0100692; 
		arm_joint_value[2] = -0.015708;
		arm_joint_value[3] = 0.0221239;
		arm_joint_value[4] = 0.110619;
	}
	else if (jointSet == mode["find_line"]){
		cout << "generate position set : find line ..." << endl;
		arm_joint_value[0] = 165 * DEG2RAD;
		arm_joint_value[1] = 100 * DEG2RAD; 
		arm_joint_value[2] = -40 * DEG2RAD;
		arm_joint_value[3] = 125 * DEG2RAD;
		arm_joint_value[4] = 165 * DEG2RAD;
	}

	return genCommand(arm_joint_value, jointNum);
}

vector< vector<int> > jointSeq(const int jointSet){
	vector< vector<int> > jSeq;

	if (jointSet == mode["home"]){
		jSeq = {{2, 3, 4}, 
  			{5}, 
                        {1}};
	}
	else if (jointSet == mode["find_line"]){
		jSeq = {{1}, 
		        {5}, 
                        {2, 3, 4}};
	}

	return jSeq;
}


int main(int argc, char **argv)
{
	// Modifidable
	if(argc != 2){
		cout << "Usage: youbot_arm_init <mode>" <<
			"\n mode :" <<
			"\n > home" <<
			"\n > find_line" << endl;
		return 1;
	}

	if( !( mode.count(argv[1]) > 0 ) ){
		cout << "mode " << argv[1] << " doesn't exists" << endl;
		cout << "Usage: youbot_arm_init <mode>" <<
			"\n mode :" <<
			"\n > home" <<
			"\n > find_line" << endl;
		return 1;
	}

	
	//int cases = find_line;
	//cout << cases << ", " << mode[cases] << endl;

	ros::init(argc, argv, "youbot_arm_init");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	armPositionsPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);

	ros::Rate rate(10);

	string cases(argv[1]);
	vector< vector<int> > jNum = jointSeq(mode[cases]);
	for(int j = 0; j < jNum.size(); j++){
		brics_actuator::JointPositions command;
		command = getPostions(mode[cases], jNum[j]);

		for(int n = 0; n < 10; n++)
			rate.sleep();

		cout << "\nsending command ..." << endl;
		armPositionsPublisher.publish(command);

		for(int n = 0; n < 15; n++)
			rate.sleep();
	}
	cout << "\ndone" << endl;
	return 0;	
}
