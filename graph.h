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

#include "plannerfunctions.h"

namespace CBFCirc
{
    class Node;
    class Edge;

    class Node
    {
    public:
        int id;
        VectorXd position;
        vector<Edge *> outEdges;
        vector<Edge *> inEdges;

        Node();
    };

    class Edge
    {
    public:
        int id;
        double weight;
        Node *nodeOut;
        Node *nodeIn;
        Matrix3d omega;
        bool forward;

        Edge();
    };

    struct NewExplorationPointResult
    {
        vector<double> value;
        vector<VectorXd> points;
        vector<int> index;
        
        VectorXd bestExplorationPosition;
        Matrix3d bestOmega;
        vector<Edge *> pathToExplorationPoint;
        bool success;
    };

    class Graph
    {
    public:
        vector<Node *> nodes;
        vector<Edge *> edges;

        Graph();
        Node *addNode(VectorXd position);
        void connect(Node *nodeOut, Node *nodeIn, double weight, Matrix3d omega);
        vector<Node *> getNeighborNodes(VectorXd position, double radius);
        Node *getNearestNode(VectorXd position);
        vector<Node *> getNearestNodeList(VectorXd position);
        NewExplorationPointResult getNewExplorationPoint(RobotPose pose, MapQuerier querier, vector<vector<VectorXd>> frontier, Parameters param);
        vector<Edge *> getPath(Node *origin, Node *target);
    };

}
