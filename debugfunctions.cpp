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
#include "controller.h"
#include "controller.cpp"
#include "debugfunctions.h"
#include "graph.h"

namespace CBFCirc
{
    string getFolderName()
    {
        time_t t = time(NULL);
        tm *timePtr = localtime(&t);

        string str = "sim_cbf_unitree_";

        str += std::to_string(timePtr->tm_mday) + "_" + std::to_string(timePtr->tm_mon + 1) + "_ts_" + std::to_string(timePtr->tm_hour) + "_" + std::to_string(timePtr->tm_min);

        return str;
    }

    void debug_Store()
    {

        if (Global::firstPlanCreated && (Global::generalCounter % Global::param.freqStoreDebug == 0))
        {
            DataForDebug dfd;

            dfd.generalCounter = Global::generalCounter;
            dfd.timeStamp = getTime();
            dfd.position = getRobotPose().position;
            dfd.orientation = getRobotPose().orientation;
            dfd.desLinVelocity = Global::desLinVelocity;
            dfd.desAngVelocity = Global::desAngVelocity;
            dfd.distance = Global::distance;
            dfd.safety = Global::safety;
            dfd.gradSafetyPosition = Global::gradSafetyPosition;
            dfd.gradSafetyOrientation = Global::gradSafetyOrientation;
            dfd.witnessDistance = Global::witnessDistance;
            dfd.currentLidarPoints = getLidarPointsKDTree(getRobotPose().position, Global::param.sensingRadius);
            dfd.currentGoalPosition = Global::currentGoalPosition;
            dfd.generateManyPathResult = Global::generateManyPathResult;
            dfd.currentOmega = Global::currentOmega;
            dfd.planningState = Global::planningState;
            dfd.graph = Global::graph;
            dfd.pointsKDTree = Global::pointsKDTree;
            dfd.pointsFrontier = getFrontierPoints(getRobotPose().position);
            // dfd.bconstraint = Global::bconstraint;

            Global::dataForDebug.push_back(dfd);
        }
    }

    void debug_addMessage(string msg)
    {
        Global::messages.push_back(std::to_string(Global::generalCounter) + ";" + msg);
    }

    void debug_generateManyPathsReport()
    {
        debug_addMessage("Omega replanned!");
        for (int k = 0; k < Global::generateManyPathResult.pathResults.size(); k++)
        {
            string pathName = getMatrixName(Global::generateManyPathResult.pathOmega[k]);
            if (Global::generateManyPathResult.pathResults[k].pathState == PathState::sucess)
                debug_addMessage("Path " + pathName + " suceeded!");

            if (Global::generateManyPathResult.pathResults[k].pathState == PathState::unfeasible)
                debug_addMessage("Path " + pathName + " unfeasible!");

            if (Global::generateManyPathResult.pathResults[k].pathState == PathState::timeout)
            {
                string errorToGoal = std::to_string(Global::generateManyPathResult.pathResults[k].finalError);
                string minimumError = std::to_string(Global::param.plannerReachError);
                debug_addMessage("Path " + pathName + " timeout! Error to path was " + errorToGoal + " but minimum is " + minimumError);
            }
        }
        if ((Global::currentOmega - Global::generateManyPathResult.bestOmega).norm())
            debug_addMessage("Changed sense of circulation");
    }

    void debug_printAlgStateToMatlab(ofstream *f)
    {

        string fname = getFolderName();

        // Update the file loader

        f->open("/home/vinicius/Desktop/matlab/unitree_planning/fileloader.m", ofstream::trunc);

        // Write to load the files
        *f << "clc;" << std::endl;
        *f << "clear all;" << std::endl;
        *f << "dirData = '" << fname << "';" << std::endl;
        *f << "timeStamp = load([dirData '/timeStamp.csv']);" << std::endl;
        *f << "generalCounter = load([dirData '/generalCounter.csv']);" << std::endl;
        *f << "position = load([dirData '/position.csv']);" << std::endl;
        *f << "orientation = load([dirData '/orientation.csv']);" << std::endl;
        *f << "desLinVelocity = load([dirData '/desLinVelocity.csv']);" << std::endl;
        *f << "desAngVelocity = load([dirData '/desAngVelocity.csv']);" << std::endl;
        *f << "distance = load([dirData '/distance.csv']);" << std::endl;
        *f << "safety = load([dirData '/safety.csv']);" << std::endl;
        *f << "gradSafetyPosition = load([dirData '/gradSafetyPosition.csv']);" << std::endl;
        *f << "gradSafetyOrientation = load([dirData '/gradSafetyOrientation.csv']);" << std::endl;
        *f << "witnessDistance = load([dirData '/witnessDistance.csv']);" << std::endl;
        *f << "currentGoalPosition = load([dirData '/currentGoalPosition.csv']);" << std::endl;
        *f << "currentLidarPoints = processCell(load([dirData '/currentLidarPoints.csv']));" << std::endl;
        *f << "currentOmega = load([dirData '/currentOmega.csv']);" << std::endl;
        *f << "planningState = load([dirData '/planningState.csv']);" << std::endl;
        *f << "graphNodes = processCell(load([dirData '/graphNodes.csv']));" << std::endl;
        *f << "graphEdges = processCell(load([dirData '/graphEdges.csv']));" << std::endl;
        *f << "pointsKDTree = processCell(load([dirData '/pointsKDTree.csv']));" << std::endl;
        *f << "pointsFrontier = processCell(load([dirData '/pointsFrontier.csv']));" << std::endl;
        *f << "messages = readtable([dirData '/messages.csv']);"<< std::endl;
        //*f << "bconstraint = load([dirData '/bconstraint.csv']);" << std::endl;

        // Write planned paths
        vector<string> names = {};
        for (int k = 0; k < Global::generateManyPathResult.pathOmega.size(); k++)
            names.push_back(getMatrixName(Global::generateManyPathResult.pathOmega[k]));

        for (int k = 0; k < names.size(); k++)
        {
            *f << "plannedPos" << names[k] << " = processCell(load([dirData '/plannedPos" << names[k] << ".csv']));" << std::endl;
            *f << "plannedOri" << names[k] << " = processCell(load([dirData '/plannedOri" << names[k] << ".csv']));" << std::endl;
            *f << "plannedGradSafPos" << names[k] << " = processCell(load([dirData '/plannedGradSafPos" << names[k] << ".csv']));" << std::endl;
            *f << "plannedGradSafOri" << names[k] << " = processCell(load([dirData '/plannedGradSafOri" << names[k] << ".csv']));" << std::endl;
            *f << "plannedDistance" << names[k] << " = processCell(load([dirData '/plannedDistance" << names[k] << ".csv']));" << std::endl;
        }

        *f << "run " << fname << "/parameters.m;" << std::endl;
        f->flush();
        f->close();

        // Write to the parameters to a separate file
        boost::filesystem::create_directories("/home/vinicius/Desktop/matlab/unitree_planning/" + fname);

        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/parameters.m", ofstream::trunc);
        *f << "param_boundingRadius =" << Global::param.boundingRadius << ";" << std::endl;
        *f << "param_boundingHeight =" << Global::param.boundingHeight << ";" << std::endl;
        *f << "param_smoothingParam =" << Global::param.smoothingParam << ";" << std::endl;
        f->flush();
        f->close();

        // Write the data
        vector<double> tempDouble;
        vector<VectorXd> tempVector;
        vector<vector<VectorXd>> tempVectorVector;

        // WRITE: timeStamp
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/timeStamp.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back(Global::dataForDebug[i].timeStamp);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: generalCounter
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/generalCounter.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].generalCounter);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: position
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/position.csv", ofstream::trunc);
        tempVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempVector.push_back(Global::dataForDebug[i].position);

        printVectorsToCSV(f, tempVector);
        f->flush();
        f->close();

        // WRITE: orientation
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/orientation.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].orientation);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: desired linear velocity
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/desLinVelocity.csv", ofstream::trunc);
        tempVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempVector.push_back(Global::dataForDebug[i].desLinVelocity);

        printVectorsToCSV(f, tempVector);
        f->flush();
        f->close();

        // WRITE: desired angular velocity
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/desAngVelocity.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].desAngVelocity);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: distance
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/distance.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].distance);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: safety
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/safety.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].safety);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: gradient of safety on position
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/gradSafetyPosition.csv", ofstream::trunc);
        tempVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempVector.push_back(Global::dataForDebug[i].gradSafetyPosition);

        printVectorsToCSV(f, tempVector);
        f->flush();
        f->close();

        // WRITE: gradient of safety on orientation
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/gradSafetyOrientation.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].gradSafetyOrientation);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: witness points
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/witnessDistance.csv", ofstream::trunc);
        tempVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempVector.push_back(Global::dataForDebug[i].witnessDistance);

        printVectorsToCSV(f, tempVector);
        f->flush();
        f->close();

        // WRITE: current lidar points
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/currentLidarPoints.csv", ofstream::trunc);
        tempVectorVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
        {
            tempVector = {};
            for (int j = 0; j < Global::dataForDebug[i].currentLidarPoints.size(); j++)
                tempVector.push_back(Global::dataForDebug[i].currentLidarPoints[j]);

            tempVectorVector.push_back(tempVector);
        }
        printVectorVectorsToCSV(f, tempVectorVector, 3);
        f->flush();
        f->close();

        // WRITE: current goal position
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/currentGoalPosition.csv", ofstream::trunc);
        tempVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempVector.push_back(Global::dataForDebug[i].currentGoalPosition);

        printVectorsToCSV(f, tempVector);
        f->flush();
        f->close();

        // WRITE: current matrix
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/currentOmega.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)getMatrixNumber(Global::dataForDebug[i].currentOmega));

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: current state of the motion planning
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/planningState.csv", ofstream::trunc);
        tempDouble = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
            tempDouble.push_back((double)Global::dataForDebug[i].planningState);

        printVectorsToCSV(f, tempDouble);
        f->flush();
        f->close();

        // WRITE: graph nodes
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/graphNodes.csv", ofstream::trunc);
        tempVectorVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
        {
            tempVector = {};
            for (int j = 0; j < Global::dataForDebug[i].graph.nodes.size(); j++)
                tempVector.push_back(Global::dataForDebug[i].graph.nodes[j]->position);

            tempVectorVector.push_back(tempVector);
        }
        printVectorVectorsToCSV(f, tempVectorVector, 3);
        f->flush();
        f->close();

        // WRITE: graph edges
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/graphEdges.csv", ofstream::trunc);
        tempVectorVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
        {
            tempVector = {};
            for (int j = 0; j < Global::dataForDebug[i].graph.edges.size(); j++)
            {
                VectorXd edge = VectorXd::Zero(2);
                int inNode = Global::dataForDebug[i].graph.edges[j]->nodeIn->id;
                int outNode = Global::dataForDebug[i].graph.edges[j]->nodeOut->id;
                edge << (double)inNode, (double)outNode;
                tempVector.push_back(edge);
            }

            tempVectorVector.push_back(tempVector);
        }
        printVectorVectorsToCSV(f, tempVectorVector, 2);
        f->flush();
        f->close();

        // WRITE: points Kd tree
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/pointsKDTree.csv", ofstream::trunc);
        tempVectorVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
        {
            tempVector = {};
            for (int j = 0; j < Global::dataForDebug[i].pointsKDTree.size(); j++)
                tempVector.push_back(Global::dataForDebug[i].pointsKDTree[j]);

            tempVectorVector.push_back(tempVector);
        }
        printVectorVectorsToCSV(f, tempVectorVector, 3);
        f->flush();
        f->close();

        // WRITE: points frontier
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/pointsFrontier.csv", ofstream::trunc);
        tempVectorVector = {};
        for (int i = 0; i < Global::dataForDebug.size(); i++)
        {
            tempVector = {};
            for (int j = 0; j < Global::dataForDebug[i].pointsFrontier.size(); j++)
                for(int k=0; k < Global::dataForDebug[i].pointsFrontier[j].size(); k++)
                    tempVector.push_back(Global::dataForDebug[i].pointsFrontier[j][k]);

            tempVectorVector.push_back(tempVector);
        }
        printVectorVectorsToCSV(f, tempVectorVector, 3);
        f->flush();
        f->close();

        // WRITE: messages
        f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/messages.csv", ofstream::trunc);
        for (int j = 0; j < Global::messages.size(); j++)
            *f << Global::messages[j] << std::endl;
        f->flush();
        f->close();

        // // WRITE: bconstraint
        // f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/bconstraint.csv", ofstream::trunc);
        // tempVector = {};
        // for (int i = 0; i < Global::dataForDebug.size(); i++)
        //     tempVector.push_back(Global::dataForDebug[i].bconstraint);

        // printVectorsToCSV(f, tempVector);
        // f->flush();
        // f->close();

        // WRITE: planned paths
        double fat = Global::param.sampleStorePath;
        for (int k = 0; k < names.size(); k++)
        {
            f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/plannedPos" + names[k] + ".csv", ofstream::trunc);
            tempVectorVector = {};
            for (int i = 0; i < Global::dataForDebug.size(); i++)
            {
                tempVector = {};
                for (int j = 0; j < Global::dataForDebug[i].generateManyPathResult.pathResults[k].path.size() / fat; j++)
                    tempVector.push_back(Global::dataForDebug[i].generateManyPathResult.pathResults[k].path[fat * j].position);

                tempVectorVector.push_back(tempVector);
            }
            printVectorVectorsToCSV(f, tempVectorVector, 3);
            f->flush();
            f->close();

            f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/plannedOri" + names[k] + ".csv", ofstream::trunc);
            tempVectorVector = {};
            for (int i = 0; i < Global::dataForDebug.size(); i++)
            {
                tempVector = {};
                for (int j = 0; j < Global::dataForDebug[i].generateManyPathResult.pathResults[k].path.size() / fat; j++)
                {
                    VectorXd data = VectorXd::Ones(1);
                    data << Global::dataForDebug[i].generateManyPathResult.pathResults[k].path[fat * j].orientation;
                    tempVector.push_back(data);
                }

                tempVectorVector.push_back(tempVector);
            }
            printVectorVectorsToCSV(f, tempVectorVector, 1);
            f->flush();
            f->close();

            f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/plannedGradSafPos" + names[k] + ".csv", ofstream::trunc);
            tempVectorVector = {};
            for (int i = 0; i < Global::dataForDebug.size(); i++)
            {
                tempVector = {};
                for (int j = 0; j < Global::dataForDebug[i].generateManyPathResult.pathResults[k].path.size() / fat; j++)
                    tempVector.push_back(Global::dataForDebug[i].generateManyPathResult.pathResults[k].pathGradSafetyPosition[fat * j]);

                tempVectorVector.push_back(tempVector);
            }
            printVectorVectorsToCSV(f, tempVectorVector, 3);
            f->flush();
            f->close();

            f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/plannedGradSafOri" + names[k] + ".csv", ofstream::trunc);
            tempVectorVector = {};
            for (int i = 0; i < Global::dataForDebug.size(); i++)
            {
                tempVector = {};
                for (int j = 0; j < Global::dataForDebug[i].generateManyPathResult.pathResults[k].path.size() / fat; j++)
                {
                    VectorXd data = VectorXd::Ones(1);
                    data << Global::dataForDebug[i].generateManyPathResult.pathResults[k].pathGradSafetyOrientation[fat * j];
                    tempVector.push_back(data);
                }

                tempVectorVector.push_back(tempVector);
            }
            printVectorVectorsToCSV(f, tempVectorVector, 1);
            f->flush();
            f->close();

            f->open("/home/vinicius/Desktop/matlab/unitree_planning/" + fname + "/plannedDistance" + names[k] + ".csv", ofstream::trunc);
            tempVectorVector = {};
            for (int i = 0; i < Global::dataForDebug.size(); i++)
            {
                tempVector = {};
                for (int j = 0; j < Global::dataForDebug[i].generateManyPathResult.pathResults[k].path.size() / fat; j++)
                {
                    VectorXd data = VectorXd::Ones(1);
                    data << Global::dataForDebug[i].generateManyPathResult.pathResults[k].pathDistance[fat * j];
                    tempVector.push_back(data);
                }

                tempVectorVector.push_back(tempVector);
            }
            printVectorVectorsToCSV(f, tempVectorVector, 1);
            f->flush();
            f->close();
        }
    }

}
