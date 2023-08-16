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
        double boundingRadius = 0.30; // 0.3
        double boundingHeight = 1.4;
        double smoothingParam = 0.5; // 0.1 0.3

        double constantHeight = -0.1725; // 0.8
        double marginSafety = 0.4;       // 0.8
        double sensingRadius = 5.0;      // 3.0

        double gainRobotYaw = 4.0;         // 2.0 4.0
        double gainTargetController = 0.4; // 0.2
        double alphaCBFPositive = 1.0;
        double alphaCBFNegative = 6;   // 7.5
        double distanceMinBeta = 0.50; // 0.5 0.3 0.4
        double maxVelCircBeta = 1.25;  // 0.5 0.5
        double maxTotalVel = 0.3;
        double distanceMargin = 0.15; // 0.20

        double deltaTimePlanner = 0.2;   // 0.1
        double maxTimePlanner = 120;     // 50 100
        double plannerReachError = 0.50; // 0.25
        double plannerOmegaPlanReachError = 0.30; // 0.25
        double acceptableRationPlanning = 2.0;

        int freqStoreDebug = 15;
        int freqReplanPath = 250; // 500
        int freqUpdateGraph = 500;
        int freqUpdateKDTree = 50; // 100
        int freqDisplayMessage = 50;

        double noMaxIterationsCorrectPoint = 20;
        double stepCorrectPoint = 0.1;
        double radiusCreateNode = 1.5; // 0.8
        double maxTimePlanConnectNode = 50;

        double minDistFilterKDTree = 0.15; // 0.3

        int sampleStorePath = 15;

        double maxTimeSampleExploration = 80;
        int noTriesClosestPoint = 5;
        VectorXd globalTargetPosition = vec3d(7, 0, -0.1725); // vec3d(10,1,-0.1725)
    };

    struct DistanceResult
    {
        double safety;
        double distance;
        VectorXd gradSafetyPosition;
        double gradSafetyOrientation;
        VectorXd witnessDistance;
    };
    struct RadialDistanceResult
    {
        double halfSqDistance;
        VectorXd gradDistance;
    };

    struct RobotPose
    {
        VectorXd position;
        double orientation;
    };

    struct CBFCircControllerResult
    {
        VectorXd linearVelocity;
        double angularVelocity;
        DistanceResult distanceResult;
        bool feasible;
    };

    struct CBFControllerResult
    {
        VectorXd linearVelocity;
        double angularVelocity;
        DistanceResult distanceResult;
        bool feasible;
    };

    enum class PathState
    {
        sucess,
        unfeasible,
        timeout,
        empty
    };

    struct GeneratePathResult
    {
        vector<RobotPose> path;
        vector<VectorXd> pathGradSafetyPosition;
        vector<double> pathGradSafetyOrientation;
        vector<double> pathDistance;
        PathState pathState;
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
        GeneratePathResult bestPath;
    };

    struct VectorFieldResult
    {
        VectorXd linearVelocity;
        double angularVelocity;
        double distance;
        int index;
    };

    enum class MotionPlanningState
    {
        goingToGlobalGoal,
        pathToExploration,
        goingToExplore,
        planning,
        sucess,
        failure
    };

    typedef function<vector<VectorXd>(VectorXd, double)> MapQuerier;

    DistanceResult computeDist(vector<VectorXd> points, RobotPose pose, Parameters param);
    CBFCircControllerResult CBFCircController(RobotPose pose, VectorXd targetPosition, vector<VectorXd> neighborPoints, Matrix3d omega,
                                              Parameters param);
    GeneratePathResult CBFCircPlanOne(RobotPose startingPose, VectorXd targetPosition, MapQuerier querier, Matrix3d omega,
                                      double maxTime, double reachpointError, Parameters param);
    double curveLength(vector<RobotPose> posePath);
    GenerateManyPathsResult CBFCircPlanMany(RobotPose startingPose, VectorXd targetPosition, MapQuerier querier,
                                            double maxTime, double reachpointError, Parameters param);
    RadialDistanceResult computeDistRadial(vector<VectorXd> points, VectorXd position, double smoothingParam);
    VectorXd correctPoint(VectorXd point, vector<VectorXd> neighborPoints, Parameters param);
    bool pathFree(vector<RobotPose> path, MapQuerier querier, int initialIndex, int finalIndex, Parameters param);
    vector<RobotPose> generateSimplePath(vector<RobotPose> originalPath, MapQuerier querier, int initialIndex, int finalIndex, Parameters param);
    vector<RobotPose> correctPath(vector<RobotPose> originalPath, MapQuerier querier, Parameters param);
    vector<RobotPose> optimizePath(vector<RobotPose> originalPath, MapQuerier querier, Parameters param);
    vector<RobotPose> upsample(vector<RobotPose> path, double minDistPos, double minDistOri);
    VectorFieldResult vectorField(RobotPose pose, vector<RobotPose> path, Parameters param);
    CBFControllerResult CBFController(RobotPose pose, VectorXd targetLinearVelocity, double targetAngularVelocity,
                                      vector<VectorXd> neighborPoints, Parameters param);

}
