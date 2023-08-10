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
        VectorXd currentGoalPosition;
        GenerateManyPathsResult generateManyPathResult;
        Matrix3d currentOmega;
        MotionPlanningState planningState;
        Graph graph;
    };

    string getFolderName();
    
    void debug_Store();
    void debug_printAlgStateToMatlab(ofstream *f);

}
