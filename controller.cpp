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
// #include <thread>

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

vector<VectorXd> getLidarPoints(VectorXd position)
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
            //for (int k = 0; k < 100; k++)
            //{
                //double h = H * ((double)k) / 100;
                

                VectorXd p = VectorXd::Zero(3);
                p << xc + r * cos(theta), yc + r * sin(theta), Global::param.constantHeight;
                obstacle.push_back(p);
            //}
        }
    }

    return obstacle;
}

void computeVelocity(VectorXd pointTarget)
{
    vector<VectorXd> lidarPoints = getLidarPoints(getRobotPose().position);
    DistanceResult dr = computeDist(lidarPoints, getRobotPose(), Global::param);


    Global::currentLidarPoints = lidarPoints;

    VectorXd vd3d = -0.2 * (getRobotPose().position - pointTarget);
    VectorXd vd = VectorXd::Zero(2);
    vd << vd3d[0], vd3d[1];


    Global::distance = dr.distance;
    Global::safety = dr.safety;
    Global::gradSafetyPosition = dr.gradSafetyPosition;
    Global::gradSafetyOrientation = dr.gradSafetyOrientation;
    Global::witnessDistance = dr.witnessDistance;

    double theta = getRobotPose().orientation;
    double ctheta = cos(theta);
    double stheta = sin(theta);
    Vector2d dir;
    dir << ctheta, stheta;
    VectorXd normVelocity = vd.normalized();
    double wd = 3 * Global::param.gainRobotYaw * (dir[0] * normVelocity[1] - dir[1] * normVelocity[0]);

    VectorXd ud = vectorVertStack(vd, wd);

    MatrixXd H = 2 * MatrixXd::Identity(3, 3);
    VectorXd f = -2 * ud;
    MatrixXd A = vectorVertStack(dr.gradSafetyPosition, dr.gradSafetyOrientation).transpose();
    VectorXd b = VectorXd::Zero(1);

    double bm;
    if (dr.safety>0)
        bm = -0.5 * (dr.safety);
    else
        bm = -4 * (dr.safety);

    b << bm;

    VectorXd u = solveQP(H, f, A, b);
    VectorXd v = VectorXd::Zero(3);
    v << u[0], u[1], 0;

    setTwist(v, u[2]);

    Global::continueAlgorithm = pow((getRobotPose().position - pointTarget).norm(),2)>=0.7*0.7+0.8*0.8;

    ROS_INFO_STREAM("-----------------------");

    ROS_INFO_STREAM("distance = " << dr.distance);
    ROS_INFO_STREAM("safety = " << dr.safety);
    ROS_INFO_STREAM("goaldist = " << printVector(getRobotPose().position - pointTarget));
    ROS_INFO_STREAM("gradSafetyP = " << printVector(dr.gradSafetyPosition));
    ROS_INFO_STREAM("gradSafetyT = " << dr.gradSafetyOrientation);
    ROS_INFO_STREAM("linVelocity = " << printVector(v));
    ROS_INFO_STREAM("angVelocity = " << u[2]);
}



// DEBUG FUNCTION

// MAIN FUNCTIONS
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

    while (ros::ok() && Global::continueAlgorithm)
    {
        ros::spinOnce();

        if (Global::measured)
            computeVelocity(currentGoalPosition);


        Global::generalCounter++;

        debug_periodicStore();

        rate.sleep();
    }

    ofstream file;
    debug_printAlgStateToMatlab(&file);

    return 0;
}
