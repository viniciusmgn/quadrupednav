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

    CBFCircControllerResult CBFCircController(RobotPose pose, VectorXd targetPosition, MapQuerier querier, Matrix3d omega, Parameters param)
    {
        vector<VectorXd> lidarPoints = querier(pose.position, param.sensingRadius);
        DistanceResult dr = computeDist(lidarPoints, pose, param);

        VectorXd vd3d = -param.gainTargetController * (pose.position - targetPosition);
        VectorXd vd = VectorXd::Zero(2);
        vd << vd3d[0], vd3d[1];

        Vector2d dir;
        dir << cos(pose.orientation), sin(pose.orientation);
        VectorXd normVelocity = vd.normalized();
        double wd = param.gainRobotYaw * (dir[0] * normVelocity[1] - dir[1] * normVelocity[0]);

        VectorXd ud = vectorVertStack(vd, wd);

        MatrixXd H = 2 * MatrixXd::Identity(3, 3);
        VectorXd f = -2 * ud;
        MatrixXd A = vectorVertStack(dr.gradSafetyPosition, dr.gradSafetyOrientation).transpose();
        

        VectorXd b = VectorXd::Zero(1);

        double cbfConst;
        if (dr.safety > 0)
            cbfConst = -param.alphaCBFPositive * (dr.safety);
        else
            cbfConst = -param.alphaCBFNegative * (dr.safety);

        b << cbfConst;

        //Add circulation is omega != 0
        if( abs(omega(0,1)) + abs(omega(1,2)) + abs(omega(2,0)) >= VERYSMALLNUMBER)
        {
            A = vectorVertStack(A, (omega*dr.gradSafetyPosition).transpose());
            double circConst = param.maxVelCircBeta*(1-dr.safety/param.safetyMinBeta);
            b = vectorVertStack(b,circConst);
        }

        VectorXd u = solveQP(H, f, A, b);

        CBFCircControllerResult cccr;
        cccr.distanceResult = dr;
        cccr.linearVelocity = VectorXd::Zero(3);

        if (u.rows() > 0)
        {
            cccr.linearVelocity << u[0], u[1], 0;
            cccr.angularVelocity = u[2];
            cccr.feasible = true;
        }
        else
        {
            cccr.angularVelocity = 0;
            cccr.feasible = false;
        }

        return cccr;
    }

    GeneratePathResult CBFCircPlanOne(RobotPose startingPose, VectorXd targetPosition, MapQuerier querier, Matrix3d omega, double maxTime, double reachpointError, Parameters param)
    {
        GeneratePathResult gpr;
        RobotPose pose = startingPose;
        vector<RobotPose> posePath = {pose};
        double time = 0;
        bool cont = true;
        double dt;

        gpr.pathState = GeneratePathState::sucess;

        while (cont)
        {
            CBFCircControllerResult cccr = CBFCircController(pose, targetPosition, querier, omega, param);
            if (cccr.feasible)
            {
                pose.position += cccr.linearVelocity * param.deltaTimePlanner;
                pose.orientation += cccr.angularVelocity * param.deltaTimePlanner;
                time += param.deltaTimePlanner;
                posePath.push_back(pose);
                cont = (posePath[posePath.size() - 1].position - targetPosition).norm() > reachpointError;
            }
            else
            {
                cont = false;

                if (!cccr.feasible)
                    gpr.pathState = GeneratePathState::unfeasible;
            }
            cont = cont && (time < maxTime);
        }

        if (time >= maxTime)
            gpr.pathState = GeneratePathState::timeout;

        gpr.path = posePath;
        gpr.finalError = (posePath[posePath.size() - 1].position - targetPosition).norm();

        return gpr;
    }
    GenerateManyPathsResult CBFCircPlanMany(RobotPose startingPose, VectorXd targetPosition,  MapQuerier querier, double maxTime, double reachpointError, Parameters param)
    {
        GenerateManyPathsResult gmpr;

        gmpr.pathOmega = {posRotZ(), -posRotZ(), Matrix3d::Zero()};
        gmpr.pathName = {"PZ","NZ","0"};
        gmpr.pathResults = {};
        gmpr.atLeastOnePathReached = false;


        for (int i = 0; i < gmpr.pathOmega.size(); i++)
        {
            GeneratePathResult gpr = CBFCircPlanOne(startingPose, targetPosition, querier, gmpr.pathOmega[i], maxTime, reachpointError, param);
            gmpr.pathResults.push_back(gpr);
            gmpr.pathLenghts.push_back(gpr.pathState == GeneratePathState::sucess ? curveLength(gmpr.pathResults[i].path) : VERYBIGNUMBER);
            gmpr.atLeastOnePathReached = gmpr.atLeastOnePathReached || (gpr.pathState == GeneratePathState::sucess);
        }

        vector<int> ind = sortGiveIndex(gmpr.pathLenghts);

        gmpr.bestPathSize = gmpr.pathLenghts[ind[0]];
        gmpr.bestOmega = gmpr.pathOmega[ind[0]];

        return gmpr;
    }

}
