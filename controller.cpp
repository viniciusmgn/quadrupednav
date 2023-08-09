#pragma once

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <functional>
#include <boost/multi_array.hpp>
#include <typeinfo>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/filesystem.hpp>

#include "std_msgs/String.h"

#include "utils.h"
#include "utils.cpp"
#include "plannerfunctions.h"
#include "plannerfunctions.cpp"
#include "debugfunctions.h"
#include "debugfunctions.cpp"
#include "controller.h"
#include "./kdtree-cpp-master/kdtree.hpp"
#include "./kdtree-cpp-master/kdtree.cpp"
#include <thread>

using namespace std;
using namespace Eigen;
using namespace CBFCirc;

// COMMUNICATION FUNCTIONS

double getTime()
{
    return ros::Time::now().toSec() - Global::startTime;
}

RobotPose getRobotPose()
{
    RobotPose pose;
    pose.position = Global::position;
    pose.orientation = Global::orientation;
    return pose;
}

void setTwist(VectorXd linearVelocity, double angularVelocity)
{

    double theta = getRobotPose().orientation;
    double ctheta = cos(theta);
    double stheta = sin(theta);

    Vector2d velTransformed;
    velTransformed << ctheta * linearVelocity[0] + stheta * linearVelocity[1], -stheta * linearVelocity[0] + ctheta * linearVelocity[1];

    geometry_msgs::Twist twist;
    twist.linear.x = velTransformed[0];
    twist.linear.y = velTransformed[1];
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angularVelocity;

    Global::desLinVelocity = linearVelocity;
    Global::desAngVelocity = angularVelocity;
    Global::pubBodyTwist->publish(twist);
}

void setLinearVelocity(VectorXd linearVelocity)
{

    double theta = getRobotPose().orientation;
    Vector2d normVelocity = linearVelocity.normalized();
    double angularVelocity = Global::param.gainRobotYaw * (cos(theta) * normVelocity[1] - sin(theta) * normVelocity[0]);

    setTwist(linearVelocity, angularVelocity);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    Global::position << (*msg).pose.pose.position.x, (*msg).pose.pose.position.y, Global::param.constantHeight;

    double coshalfv = (*msg).pose.pose.orientation.w;
    double sinhalfv = sqrt(pow((*msg).pose.pose.orientation.x, 2) + pow((*msg).pose.pose.orientation.y, 2) + pow((*msg).pose.pose.orientation.z, 2));

    if ((*msg).pose.pose.orientation.z < 0)
        sinhalfv = -sinhalfv;

    Global::orientation = 2 * atan2(sinhalfv, coshalfv);

    Global::measured = true;
}

vector<VectorXd> getLidarPoints(VectorXd position, double radius)
{
    double R = 1;
    double xc = 4;
    double yc = 0.5;
    double H = 2;

    vector<VectorXd> obstacle;
    for (int i = 0; i <= 6; i++)
    {
        double r = R * ((double)i) / 6;
        int jmax = (i + 1) * 10;
        for (int j = 0; j < jmax; j++)
        {
            double theta = (2 * 3.14) * j / ((double)jmax);
            // for (int k = 0; k < 100; k++)
            //{
            // double h = H * ((double)k) / 100;

            VectorXd p = VectorXd::Zero(3);
            p << xc + r * cos(theta), yc + r * sin(theta), Global::param.constantHeight;
            obstacle.push_back(p);
            //}
        }
    }

    return obstacle;
}

// DEBUG FUNCTION

// MAIN FUNCTIONS

void lowLevelMovement()
{
    while (true)
    {
        if (Global::measured)
        {
            Global::currentLidarPoints = getLidarPoints(getRobotPose().position, Global::param.sensingRadius);
            CBFCircControllerResult cccr = CBFCircController(getRobotPose(), Global::currentGoalPosition,
                                                             Global::currentLidarPoints, Global::currentOmega, Global::param);

            setTwist(cccr.linearVelocity, cccr.angularVelocity);

            if (Global::generalCounter % 50 == 0)
            {
                ROS_INFO_STREAM("---------------------");
                ROS_INFO_STREAM("linvelocity = " << printVector(cccr.linearVelocity));
                ROS_INFO_STREAM("angvelocity = " << cccr.angularVelocity);
                ROS_INFO_STREAM("distobs = " << cccr.distanceResult.distance);
                ROS_INFO_STREAM("safety = " << cccr.distanceResult.safety);
                ROS_INFO_STREAM("distgoal = " << (getRobotPose().position - Global::currentGoalPosition).norm());
                ROS_INFO_STREAM("omega = " << getMatrixName(Global::currentOmega));
            }
        }
    }
}

int main(int argc, char **argv)
{

    // Initialize everything
    ros::init(argc, argv, "manager");
    ros::NodeHandle nodeHandler;

    ros::Publisher aux_pubBodyTwist = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber subPose = nodeHandler.subscribe("/odom/ground_truth", 1000, poseCallback);

    Global::pubBodyTwist = &aux_pubBodyTwist;
    Global::startTime = ros::Time::now().toSec();

    ros::Rate rate(100);

    Global::currentGoalPosition << 9, 0, 0;
    Global::currentOmega = Matrix3d::Zero();
    std::thread lowLevelMovementThread = thread(lowLevelMovement);

    while (ros::ok() && Global::continueAlgorithm)
    {
        ros::spinOnce();

        if (Global::measured)
        {
            if (Global::generalCounter % Global::param.freqReplanPath == 0)
            {
                Global::generateManyPathResult = CBFCircPlanMany(getRobotPose(), Global::currentGoalPosition, getLidarPoints,
                                                                 Global::param.maxTimePlanner, Global::param.plannerReachError, Global::param);
                Global::currentOmega = Global::generateManyPathResult.bestOmega;
            }

            Global::generalCounter++;

            debug_periodicStore();

            Global::continueAlgorithm = Global::generalCounter < 3000;
        }

        rate.sleep();
    }

    ofstream file;
    debug_printAlgStateToMatlab(&file);

    return 0;
}
