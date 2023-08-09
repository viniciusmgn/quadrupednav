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

#include "plannerfunctions.h"
#include "utils.h"

namespace CBFCirc
{
    VectorFieldResult vectorField(VectorXd point, vector<VectorXd> path, double alpha, double percentLengthStop)
    {

        // Find the closest point in the curve
        double dmin = VERYBIGNUMBER;
        double dminTemp;
        int ind = 0;
        vector<double> s;
        s.push_back(0);

        for (int i = 0; i < path.size(); i++)
        {
            dminTemp = (path[i] - point).norm();
            if (dminTemp < dmin)
            {
                dmin = dminTemp;
                ind = i;
            }
            if (i > 0)
                s.push_back(s[s.size() - 1] + (path[i] - path[i - 1]).norm());
        }

        VectorFieldResult vfr;
        vfr.distance = dmin;
        vfr.index = ind;

        if (path.size() < 5)
        {
            vfr.vector = 0.2 * (path[path.size() - 1] - point).normalized();
        }
        else
        {
            VectorXd pi = path[ind];

            // Compute the normal vector
            VectorXd N = (pi - point) / ((pi - point).norm() + VERYSMALLNUMBER);

            // Compute the tangent vector
            VectorXd T;

            if (ind == 0)
                T = (path[1] - path[0]).normalized();
            else
                T = (path[ind] - path[ind - 1]).normalized();

            // Compute the G and H gains
            double G = (2 / M_PI) * atan(alpha * sqrt(dmin));
            double H = sqrt(1 - (1 - VERYSMALLNUMBER) * G * G);

            // Compute the final vector field:
            VectorXd v = G * N + H * T;

            // Scale if the curve is open
            double sCurrent = s[ind];
            double sMaximum = s[s.size() - 1];
            double mult;

            // mult = sqrt(abs((1 - (sCurrent - percentLengthStop * sMaximum) / ((1 - percentLengthStop) * sMaximum))));
            // if (sCurrent > percentLengthStop * sMaximum)
            //     v = mult * v;

            vfr.vector = v;
        }
        return vfr;
    }
    DistanceResult computeDist(vector<VectorXd> points, RobotPose pose, Parameters param)
    {
        DistanceResult dr;
        vector<double> safety;
        vector<VectorXd> gradientsPosition;
        vector<VectorXd> gradientsOrientation;

        double cosang = cos(pose.orientation);
        double sinang = sin(pose.orientation);
        double x, y, z;

        dr.distance = VERYBIGNUMBER;

        for (int i = 0; i < points.size(); i++)
        {
            // Transform point
            VectorXd pointTrans = VectorXd::Zero(3);
            x = points[i][0] - pose.position[0];
            y = points[i][1] - pose.position[1];
            z = points[i][2] - pose.position[2];
            pointTrans << cosang * x + sinang * y, -sinang * x + cosang * y, z;
            SafetyResult sr = safetyCylinder(pointTrans, param.boundingRadius, param.boundingHeight);

            // Compute safety
            safety.push_back(sr.safety);

            // Compute distance and the witness points
            double tempDist = signedDistCylinder(pointTrans, param.boundingRadius, param.boundingHeight);
            if (tempDist < dr.distance)
            {
                dr.distance = tempDist;
                dr.witnessDistance = points[i];
            }

            // Compute gradient of the safety on position
            VectorXd gradSafetyPosition = VectorXd::Zero(3);
            x = sr.gradSafety[0];
            y = sr.gradSafety[1];
            z = sr.gradSafety[2];
            gradSafetyPosition << -(cosang * x - sinang * y), -(sinang * x + cosang * y), -z;
            gradientsPosition.push_back(gradSafetyPosition);

            // Compute gradient of the safety on orientation
            VectorXd pointRotated = VectorXd::Zero(3);
            pointRotated << pointTrans[1], -pointTrans[0], 0;
            gradientsOrientation.push_back(sr.gradSafety.transpose() * pointRotated);
        }

        SoftSelectMinResult ssmrPosition = softSelectMin(3, safety, gradientsPosition, param.smoothingParam);
        SoftSelectMinResult ssmrOrientation = softSelectMin(1, safety, gradientsOrientation, param.smoothingParam);

        dr.safety = ssmrPosition.softMin - param.marginSafety;
        dr.gradSafetyPosition = ssmrPosition.selected;
        dr.gradSafetyOrientation = ssmrOrientation.selected[0];

        return dr;
    }

    CBFCircControllerResult CBFCircController(RobotPose startingPose, VectorXd targetPosition, MapQuerier querier, Matrix3d omega, Parameters param)
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
        if (dr.safety > 0)
            bm = -0.5 * (dr.safety);
        else
            bm = -4 * (dr.safety);

        b << bm;

        VectorXd u = solveQP(H, f, A, b);
        VectorXd v = VectorXd::Zero(3);
        v << u[0], u[1], 0;

        setTwist(v, u[2]);

        Global::continueAlgorithm = pow((getRobotPose().position - pointTarget).norm(), 2) >= 0.7 * 0.7 + 0.8 * 0.8;

        ROS_INFO_STREAM("-----------------------");

        ROS_INFO_STREAM("distance = " << dr.distance);
        ROS_INFO_STREAM("safety = " << dr.safety);
        ROS_INFO_STREAM("goaldist = " << printVector(getRobotPose().position - pointTarget));
        ROS_INFO_STREAM("gradSafetyP = " << printVector(dr.gradSafetyPosition));
        ROS_INFO_STREAM("gradSafetyT = " << dr.gradSafetyOrientation);
        ROS_INFO_STREAM("linVelocity = " << printVector(v));
        ROS_INFO_STREAM("angVelocity = " << u[2]);
    }

}
