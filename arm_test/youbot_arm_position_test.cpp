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

#define NumOfJoint = 5;
#define DEG2RAD 0.01745329
#define RAD2DEG 57.29578049

class ArmSubTest
{
public:
	ArmSubTest();
	~ArmSubTest();
private:
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Subscriber jointStateSubscriber;

	const string arm_joint_name[NumOfJoint] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
	double arm_joint_state[NumOfJoint] = { 0.0, 0.0, 0.0, 0.0, 0.0 }; //radian
	double arm_joint_max[NumOfJoint] = { 334.615, 149.999, -0.90001, 196.478, 323.239 };	//degree
	double arm_joint_min[NumOfJoint] = { 0.577  , 0.577  , -288    , 1.268  , 6.33801 };	//degree

	int mode = 0;

	void startLoop();
	void showJointState();
	void jointCallback(const sensor_msgs::JointState &jointStates);
	void setSingleJoint(const int joint_num, const double joint_value);
};

// ===== Initialization =====================================================================

ArmSubTest::ArmSubTest(){
	armPositionsPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1, this);
	jointStateSubscriber = n.subscribe("/joint_states", 1, &ArmSubTest::jointCallback, this);
	this -> startLoop();
}
ArmSubTest::~ArmSubTest(){}

// ===== Subscriber : Get the joint angles from the encoder =================================

void ArmSubTest::jointCallback(const sensor_msgs::JointState &jointStates){
	// cout << "===== Subscriber Callback =====" << endl;
	for (int i = 0; i < jointStates.position.size(); i++){
		for (int j = 0; j < NumOfJoint; j++){
			if (jointStates.name[i] == arm_joint_name[j]){
				arm_joint_state[j] = jointStates.position[i];
				//cout << "Joint #" << j << " : " << jointStates.name[j] << jointStates.position[i] << " => " << arm_joint_state[j] << endl; 
				break;
			}
		}
	} 
}

void ArmSubTest::showJointState(){

	cout << "===== Joint State =====" << endl;
	cout <<   "Joint #1: " << arm_joint_state[0]*RAD2DEG << 
		"\nJoint #2: " << arm_joint_state[1]*RAD2DEG <<
		"\nJoint #3: " << arm_joint_state[2]*RAD2DEG <<
		"\nJoint #4: " << arm_joint_state[3]*RAD2DEG <<
		"\nJoint #5: " << arm_joint_state[4]*RAD2DEG << endl;

}

// ===== Publisher : Set desired joint angles respectively =============================================

void ArmSubTest::setSingleJoint(const int joint_num, const double joint_value){
	brics_actuator::JointPositions command;
	vector<brics_actuator::JointValue> armJointPositions;
	armJointPositions.resize(1);	
	
	armJointPositions[0].joint_uri = arm_joint_name[joint_num - 1];
	armJointPositions[0].unit = boost::units::to_string(boost::units::si::radians);
	armJointPositions[0].value = joint_value;

	cout << "Single Joint Setting: " << armJointPositions[0].joint_uri << ", " << armJointPositions[0].value << endl;
	cout << "sending command ..." << endl;
	
	command.positions = armJointPositions;
	armPositionsPublisher.publish(command);
}

// ===== Main Processing Block =============================================================================

void ArmSubTest::startLoop(){
	
	while(n.ok()){
		ros::spinOnce();

		cout << "\n===== Choose mode =====" <<
			"\n>  0: Exit" <<
			"\n>  1: Show Joint State" <<
			"\n>  2: Set Joint Position (Single)" << endl;
		
		cin >> mode;
		while(mode < 0 || mode > 3){
			cout << "[WRONG] The mode number is out of range." <<
				"\n===== Choose mode =====" <<
				"\n>  0: Exit" <<
				"\n>  1: Show Joint State" <<
				"\n>  2: Set Joint Position (Single)" << endl;
			cin >> mode;
		}
		
		if(mode == 0){
			cout << "exit..." << endl;
			break;		
		}
		/* Mode 1 Show Joint State(s) */
		else if(mode == 1){
			ros::spinOnce();
			this -> showJointState();
		}
		/* Mode 2 Set Single Joint Value */
		else if(mode == 2){
			ros::spinOnce();
			int joint_num = 0;
			double joint_val;
			cout << "[Set Joint Position (Single)] \nChoose Joint: (1~5)" << endl;
			cin >> joint_num;
			if(joint_num < 1 || joint_num > 5){
				cout << "Joint number out of range (#1~#5)" << endl;
				continue;
			}
			else{
				cout << "Desired joint value (deg.): " << endl;
				cin >> joint_val;
				if(joint_val >= arm_joint_max[joint_num-1] || joint_val <= arm_joint_min[joint_num-1]){
					cout << "Joint#" << joint_num << " : " << arm_joint_min[joint_num] << " ~ " << arm_joint_max[joint_num] << " (deg.)" << "\nDesired joint value:" << endl;
					continue;
				}				
				else
					this -> setSingleJoint(joint_num, joint_val * DEG2RAD);  // radian
			}
		}		
		/* Else : do nth. */
		else
			continue;
	}// End of While
}// End of StartLoop()

int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_arm_position_test");
	ArmSubTest Arm_Position_SubTest;
	return 0;	
}
