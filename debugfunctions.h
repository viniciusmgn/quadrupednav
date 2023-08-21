#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <numeric>
#include <algorithm>

using namespace std;
using namespace Eigen;

#include "graph.h"



namespace CBFCirc
{
    class DataForDebug
    {
    public:
        double timeStamp;
        int generalCounter;
        Vector3d position;
        double orientation;
        VectorXd desLinVelocity;
        double desAngVelocity;
        double distance;
        double safety;
        VectorXd gradSafetyPosition;
        double gradSafetyOrientation;
        VectorXd witnessDistance;
        vector<VectorXd> currentLidarPoints;
        vector<VectorXd> currentLidarPointsKDTree;
        VectorXd currentGoalPosition;
        GenerateManyPathsResult generateManyPathResult;
        Matrix3d currentOmega;
        MotionPlanningState planningState;
        Graph graph;
        vector<VectorXd> pointsKDTree;
        vector<vector<VectorXd>> pointsFrontier;
        vector<Edge *> currentPath;
        int currentIndexPath;
        VectorXd explorationPosition;
        vector<RobotPose> commitedPath;

    };

    string getFolderName();
    void debug_addMessage(int counter, string msg);
    void debug_Store(int counter);
    void debug_generateManyPathsReport(int counter);
    void debug_printAlgStateToMatlab(ofstream *f);
    

}
