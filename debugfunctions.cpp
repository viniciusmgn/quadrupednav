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
#include "debugfunctions.h"

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

    void debug_periodicStore()
    {

        if (Global::generalCounter % Global::param.freqStoreDebug == 0)
        {
            DataForDebug dfd;

            dfd.generalCounter = Global::generalCounter;
            dfd.timeStamp = getTime();
            dfd.position = getRobotPose().position;
            dfd.orientation = getRobotPose().orientation;
            dfd.messages = {};
            dfd.messages.push_back("Periodic storage");
            dfd.desLinVelocity = Global::desLinVelocity;
            dfd.desAngVelocity = Global::desAngVelocity;
            dfd.distance = Global::distance;
            dfd.safety = Global::safety;
            dfd.gradSafetyPosition = Global::gradSafetyPosition;
            dfd.gradSafetyOrientation = Global::gradSafetyOrientation;
            dfd.witnessDistance = Global::witnessDistance;
            dfd.currentLidarPoints = Global::currentLidarPoints;
            

            Global::dataForDebug.push_back(dfd);
        }
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
        *f << "currentLidarPoints = processCell(load([dirData '/currentLidarPoints.csv']));"<< std::endl;

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
        printVectorVectorsToCSV(f, tempVectorVector);
        f->flush();
        f->close();
    }

}
