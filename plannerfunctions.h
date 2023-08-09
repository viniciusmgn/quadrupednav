#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>

using namespace std;
using namespace Eigen;

namespace CBFCirc
{

    struct Parameters
    {
        double boundingRadius = 0.5;
        double boundingHeight = 1.4;
        double smoothingParam = 0.1;
        double gainRobotYaw = 0.5;
        double constantHeight = 0.8;
        double marginSafety = 0.8;


        int freqStoreDebug = 3;
    };

    struct DistanceResult
    {
        double safety;
        double distance;
        VectorXd gradSafetyPosition;
        double gradSafetyOrientation;
        VectorXd witnessDistance;
    };

    struct RobotPose
    {
        VectorXd position;
        double orientation;
    };

    struct VectorFieldResult
    {
        VectorXd vector;
        double distance;
        int index;
    };

    struct CBFCircControllerResult
    {
        VectorXd linearVelocity;
        double angularVelocity;
        double safety;
        double distance;
        bool feasible;
    };

    typedef function<vector<VectorXd>(VectorXd, double)> MapQuerier;


    DistanceResult computeDist(vector<VectorXd> points, RobotPose pose, Parameters param);
    VectorFieldResult vectorField(VectorXd point, vector<VectorXd> path, double alpha, double percentLengthStop);
    CBFCircControllerResult CBFCircController(RobotPose pose, VectorXd targetPosition, MapQuerier querier, Matrix3d omega, Parameters param);





}
