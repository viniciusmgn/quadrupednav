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
        double boundingRadius = 0.3;
        double boundingHeight = 1.4;
        double smoothingParam = 0.1;
        
        double constantHeight = 0.8;
        double marginSafety = 0.4; //0.8
        double sensingRadius = 3.0;

        double gainRobotYaw = 2.0;
        double gainTargetController = 0.2;
        double alphaCBFPositive = 0.5;
        double alphaCBFNegative = 0.5;
        double safetyMinBeta = 0.3; //0.5
        double maxVelCircBeta = 0.25; //0.5
        double maxTotalVel = 0.3;
        
        double deltaTimePlanner=0.1;
        double maxTimePlanner=50;
        double plannerReachError = 0.2;

        int freqStoreDebug = 15;
        int freqReplanPath = 500;
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
        DistanceResult distanceResult;
        bool feasible;
    };

    enum GeneratePathState
    {
        sucess,
        unfeasible,
        timeout,
        empty
    };

    struct GeneratePathResult
    {
        vector<RobotPose> path;
        GeneratePathState pathState;
        double finalError;
    };

    struct GenerateManyPathsResult
    {
        vector<GeneratePathResult> pathResults;
        vector<Matrix3d> pathOmega;
        vector<double> pathLenghts;
        bool atLeastOnePathReached;
        double bestPathSize;
        Matrix3d bestOmega;
    };

    typedef function<vector<VectorXd>(VectorXd, double)> MapQuerier;


    DistanceResult computeDist(vector<VectorXd> points, RobotPose pose, Parameters param);
    VectorFieldResult vectorField(VectorXd point, vector<VectorXd> path, double alpha, double percentLengthStop);
    CBFCircControllerResult CBFCircController(RobotPose pose, VectorXd targetPosition, vector<VectorXd> lidarPoints, Matrix3d omega, Parameters param);
    GeneratePathResult CBFCircPlanOne(RobotPose startingPose, VectorXd targetPosition,  MapQuerier querier, Matrix3d omega, double maxTime, double reachpointError, Parameters param); 
    double curveLength(vector<RobotPose> posePath);
    GenerateManyPathsResult CBFCircPlanMany(RobotPose startingPose, VectorXd targetPosition,  MapQuerier querier, double maxTime, double reachpointError, Parameters param);




}
