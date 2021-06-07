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

void positionCB(const brics_actuator::JointPositions &msg)
{	
    vector<brics_actuator::JointValue> posCommands = msg.positions;
    int size = posCommands.size();

    cout << "------------------------------------------\n" <<	
            "Position Command Received !!!\n" << endl;

    for(int n = 0; n < size; n++)
    {
        cout << "[ " << posCommands[n].joint_uri << " ] : " 
             << setw(10) << posCommands[n].value 
             << " rad" << endl;
    }
    cout << "\nDone." << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_arm_position_listener");
    ros::NodeHandle n;
    ros::Subscriber posCommandSubscriber;	

    posCommandSubscriber = n.subscribe("arm_1/arm_controller/position_command", 1, &positionCB);
    ros::spin();
    return 0;
}
