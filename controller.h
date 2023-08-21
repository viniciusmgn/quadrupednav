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
#include <mutex>
#include <shared_mutex>
#include "std_msgs/String.h"
#include "./kdtree-cpp-master/kdtree.hpp"




#include "utils.h"
#include "utils.cpp"
#include "plannerfunctions.h"
#include "plannerfunctions.cpp"
#include "debugfunctions.h"
#include "debugfunctions.cpp"
#include "graph.h"
#include "graph.cpp"
// #include <thread>

using namespace std;
using namespace Eigen;
using namespace CBFCirc;

// STRUCT AND CLASSES




class Global
{
public:
    inline static double startTime = 0;
    inline static VectorXd position = VectorXd::Zero(3);
    inline static double orientation = 0;
    inline static VectorXd desLinVelocity = VectorXd::Zero(3);
    inline static double desAngVelocity = 0;
    inline static ros::Publisher *pubBodyTwist = NULL;
    inline static int generalCounter = 0;
    inline static bool measured = false;
    inline static double distance = 0;
    inline static double safety = 0;
    inline static VectorXd gradSafetyPosition = VectorXd::Zero(3);
    inline static double gradSafetyOrientation = 0;
    inline static VectorXd witnessDistance = VectorXd::Zero(3);
    inline static bool continueAlgorithm = true;
    inline static VectorXd currentGoalPosition = VectorXd::Zero(3);
    inline static GenerateManyPathsResult generateManyPathResult;
    inline static Matrix3d currentOmega;
    inline static MotionPlanningState planningState = MotionPlanningState::goingToGlobalGoal;
    inline static bool firstPlanCreated = false;
    inline static vector<string> messages = {};
    inline static Graph graph;
    inline static Kdtree::KdTree *kdTree;
    inline static vector<VectorXd> pointsKDTree = {};
    inline static shared_timed_mutex mutexUpdateKDTree;
    inline static mutex mutexReplanCommitedPath;
    inline static mutex mutexUpdateGraph;
    inline static vector<DataForDebug> dataForDebug = {};
    inline static Parameters param;
    inline static vector<Edge *> currentPath = {};
    inline static int currentIndexPath = -1;
    inline static VectorXd explorationPosition = VectorXd::Zero(3);  
    inline static ros::ServiceClient* neighborhClient;
    inline static ros::ServiceClient* frontierClient;
    inline static tf::TransformListener* tflistener;
    inline static tf::StampedTransform* transform;
    inline static vector<RobotPose> commitedPath;
    inline static double measuredHeight;

};



double getTime();
RobotPose getRobotPose();
vector<VectorXd> getLidarPointsKDTree(VectorXd position, double radius);
vector<VectorXd> getLidarPointsSource(VectorXd position, double radius);
vector<vector<VectorXd>> getFrontierPoints();
void updateGraphCall();
void updateKDTreeCall();

