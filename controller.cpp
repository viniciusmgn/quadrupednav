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
#include <mutex>

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

    //double newOrientation = 2 * atan2(sinhalfv, coshalfv);
    //Global::orientation = 0.9*Global::orientation +  0.1*newOrientation;
    Global::orientation = 2 * atan2(sinhalfv, coshalfv);

    Global::measured = true;
}

vector<VectorXd> createRectangle(double xcenter, double ycenter, double xlength, double ylength, double angRot)
{
    double SEP = 0.3;
    vector<VectorXd> points = {};

    double C = cos(angRot);
    double S = sin(angRot);

    for (int i = 0; i < round(xlength / SEP); i++)
        for (int j = 0; j < round(ylength / SEP); j++)
        {
            double x = -xlength / 2 + SEP * i;
            double y = -ylength / 2 + SEP * j;
            double xmod = C * x - S * y + xcenter;
            double ymod = -S * x + C * y + ycenter;
            VectorXd p = VectorXd::Zero(3);
            p << xmod, ymod, Global::param.constantHeight;

            points.push_back(p);
        }

    return points;
}

vector<VectorXd> getLidarPoints(VectorXd position, double radius)
{
    vector<VectorXd> obstacle = {};

    // double R = 1;
    // double xc = 4;
    // double yc = 0.5;
    // double H = 2;

    // for (int i = 0; i <= 6; i++)
    // {
    //     double r = R * ((double)i) / 6;
    //     int jmax = (i + 1) * 7;
    //     for (int j = 0; j < jmax; j++)
    //     {
    //         double theta = (2 * 3.14) * j / ((double)jmax);
    //         VectorXd p = VectorXd::Zero(3);
    //         p << xc + r * cos(theta), yc + r * sin(theta), Global::param.constantHeight;
    //         obstacle.push_back(p);
    //     }
    // }
    vector<VectorXd> obs1 = createRectangle(4, 1.5 + 0.8 + 4, 1, 8, 0);
    vector<VectorXd> obs2 = createRectangle(4, 1.5 - 0.8 - 4, 1, 8, 0);

    vector<VectorXd> obs3 = createRectangle(6.5, -1.5 + 0.8 + 7, 1, 14, 0);
    vector<VectorXd> obs4 = createRectangle(6.5, -1.5  -0.8 - 3, 1, 6, 0);

    vector<VectorXd> obs5 = createRectangle(1, -6.8, 10, 1, 0);
    vector<VectorXd> obs6 = createRectangle(1,  10, 10, 1, 0);

    for (int i = 0; i < obs1.size(); i++)
        obstacle.push_back(obs1[i]);

    for (int i = 0; i < obs2.size(); i++)
        obstacle.push_back(obs2[i]);

    for (int i = 0; i < obs3.size(); i++)
        obstacle.push_back(obs3[i]);

    for (int i = 0; i < obs4.size(); i++)
        obstacle.push_back(obs4[i]);

    for (int i = 0; i < obs5.size(); i++)
        obstacle.push_back(obs5[i]);

    for (int i = 0; i < obs6.size(); i++)
        obstacle.push_back(obs6[i]);

    return obstacle;
}

// MAIN FUNCTIONS

void lowLevelMovement()
{
    while (ros::ok() && Global::continueAlgorithm)
    {
        if (Global::measured && Global::firstPlanCreated)
        {
            vector<VectorXd> obsPoints = getLidarPoints(getRobotPose().position, Global::param.sensingRadius);
            CBFCircControllerResult cccr = CBFCircController(getRobotPose(), Global::currentGoalPosition,
                                                             obsPoints, Global::currentOmega, Global::param);

            // Refresh some variables
            Global::distance = cccr.distanceResult.distance;
            Global::safety = cccr.distanceResult.safety;
            Global::gradSafetyPosition = cccr.distanceResult.gradSafetyPosition;
            Global::gradSafetyOrientation = cccr.distanceResult.gradSafetyOrientation;
            Global::witnessDistance = cccr.distanceResult.witnessDistance;

            // Send the twist
            setTwist(cccr.linearVelocity, cccr.angularVelocity);
        }
    }
}
void replanOmega()
{
    while (ros::ok() && Global::continueAlgorithm)
    {
        if (Global::measured && (Global::generalCounter % Global::param.freqReplanPath == 0))
        {
            Global::generateManyPathResult = CBFCircPlanMany(getRobotPose(), Global::currentGoalPosition, getLidarPoints,
                                                             Global::param.maxTimePlanner, Global::param.plannerReachError, Global::param);
            Global::currentOmega = Global::generateManyPathResult.bestOmega;
            Global::firstPlanCreated = true;
        }
    }
}
void updateGraph()
{
    while (ros::ok() && Global::continueAlgorithm)
    {
        if (Global::measured && (Global::generalCounter % Global::param.freqUpdateGraph == 0))
        {
            VectorXd currentPoint = getRobotPose().position;
            VectorXd correctedPoint = correctPoint(currentPoint, getLidarPoints(getRobotPose().position,
                                                   Global::param.sensingRadius), Global::param);

            if (Global::graph.getNeighborNodes(correctedPoint, Global::param.radiusCreateNode).size() == 0)
            {
                vector<double> distances;
                vector<int> indexes;
                vector<Matrix3d> omegas;

                for (int i = 0; i < Global::graph.nodes.size(); i++)
                {
                    RobotPose pose;
                    pose.position = Global::graph.nodes[i]->position;
                    pose.orientation = 0;

                    GenerateManyPathsResult gmpr = CBFCircPlanMany(pose, correctedPoint, getLidarPoints,
                                                                   Global::param.maxTimePlanConnectNode, Global::param.plannerReachError, Global::param);
                    if (gmpr.atLeastOnePathReached)
                    {
                        indexes.push_back(i);
                        omegas.push_back(gmpr.bestOmega);
                        distances.push_back(gmpr.bestPathSize);
                    }
                }

                if (distances.size() > 0)
                {
                    vector<int> ind = sortGiveIndex(distances);
                    Node *newNode = Global::graph.addNode(correctedPoint);
                    Global::graph.connect(Global::graph.nodes[indexes[ind[0]]], newNode, distances[ind[0]], omegas[ind[0]]);
                }
            }
        }
    }
}

int main(int argc, char **argv)
{

    // Initialize ROS variables
    ros::init(argc, argv, "manager");
    ros::NodeHandle nodeHandler;
    ros::Publisher aux_pubBodyTwist = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber subPose = nodeHandler.subscribe("/odom/ground_truth", 1000, poseCallback);
    ros::Rate rate(100);

    // Initialize some global variables
    Global::pubBodyTwist = &aux_pubBodyTwist;
    Global::startTime = ros::Time::now().toSec();
    Global::currentGoalPosition << 9, 1, 0.8;
    Global::currentOmega = Matrix3d::Zero();
    VectorXd startingPosition = VectorXd::Zero(3);
    startingPosition << 0, 0, Global::param.constantHeight;
    Global::graph.addNode(startingPosition);

    // Initialize some threads
    std::thread lowLevelMovementThread = thread(lowLevelMovement);
    std::thread replanOmegaThread = thread(replanOmega);
    std::thread updateGraphThread = thread(updateGraph);

    while (ros::ok() && Global::continueAlgorithm)
    {
        ros::spinOnce();

        if (Global::measured)
        {

            Global::generalCounter++;

            if (Global::generalCounter % 50 == 0)
            {
                if (Global::planningState == MotionPlanningState::goingToGlobalGoal)
                {
                    ROS_INFO_STREAM("-----GOING TO GLOBAL TARGET------");
                }
                if (Global::planningState == MotionPlanningState::pathToExploration)
                {
                    ROS_INFO_STREAM("-------PATH TO EXPLORATION-------");
                }
                if (Global::planningState == MotionPlanningState::goingToExplore)
                {
                    ROS_INFO_STREAM("---------GOING TO EXPLORE--------");
                }
                // ROS_INFO_STREAM("counter = " << Global::generalCounter);
                // ROS_INFO_STREAM("linvelocity = " << printVector(Global::desLinVelocity));
                // ROS_INFO_STREAM("angvelocity = " << Global::desAngVelocity);
                ROS_INFO_STREAM("distobs = " << Global::distance);
                ROS_INFO_STREAM("safety = " << Global::safety);
                ROS_INFO_STREAM("distgoal = " << (getRobotPose().position - Global::currentGoalPosition).norm());
                ROS_INFO_STREAM("omega = " << getMatrixName(Global::currentOmega));
            }

            // DEBUG
            Global::messages.push_back(std::to_string(Global::generalCounter) + ": periodic storage");
            debug_Store();
            // DEBUG

            Global::continueAlgorithm = (getRobotPose().position - Global::currentGoalPosition).norm() >= 0.3;
        }

        rate.sleep();
    }

    lowLevelMovementThread.join();
    replanOmegaThread.join();
    updateGraphThread.join();

    ofstream file;
    debug_printAlgStateToMatlab(&file);

    return 0;
}
