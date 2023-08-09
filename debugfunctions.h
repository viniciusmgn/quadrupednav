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
        vector<string> messages;
        double distance;
        double safety;
        VectorXd gradSafetyPosition;
        double gradSafetyOrientation;
        VectorXd witnessDistance;
        vector<VectorXd> currentLidarPoints;
        VectorXd currentGoalPosition;
        GenerateManyPathsResult generateManyPathResult;
        Matrix3d currentOmega;
    };

    string getFolderName();
    
    void debug_periodicStore();
    void debug_printAlgStateToMatlab(ofstream *f);

}
