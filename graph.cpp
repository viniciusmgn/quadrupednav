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

#include "graph.h"
#include "utils.h"
#include "plannerfunctions.h"

namespace CBFCirc
{

    Node::Node()
    {
        this->id = 0;
        this->position = VectorXd::Zero(3);
        this->outEdges = {};
        this->inEdges = {};
    };

    Edge::Edge()
    {
        this->id = 0;
        this->weight = 0;
        this->nodeIn = NULL;
        this->nodeOut = NULL;
        this->omega = Matrix3d::Zero();
    };

    Graph::Graph()
    {
        this->nodes = {};
        this->edges = {};
    }

    Node *Graph::addNode(VectorXd position)
    {

        Node *newNode = new Node;
        newNode->id = nodes.size();
        newNode->position = position;
        nodes.push_back(newNode);

        Edge *selfEdge = new Edge;
        selfEdge->id = edges.size();
        selfEdge->nodeOut = newNode;
        selfEdge->nodeIn = newNode;
        selfEdge->omega = Matrix3d::Zero();
        selfEdge->weight = 0;
        selfEdge->forward = true;
        edges.push_back(selfEdge);

        return newNode;
    }

    void Graph::connect(Node *nodeOut, Node *nodeIn, double weight, Matrix3d omega)
    {
        // Create forward and reverse edges

        Edge *newEdgeForward = new Edge;
        newEdgeForward->id = edges.size();
        newEdgeForward->nodeOut = nodeOut;
        newEdgeForward->nodeIn = nodeIn;
        newEdgeForward->omega = omega;
        newEdgeForward->weight = weight;
        nodeOut->outEdges.push_back(newEdgeForward);
        nodeIn->inEdges.push_back(newEdgeForward);
        newEdgeForward->forward = true;

        edges.push_back(newEdgeForward);

        Edge *newEdgeReverse = new Edge;
        newEdgeReverse->id = edges.size();
        newEdgeReverse->nodeOut = nodeIn;
        newEdgeReverse->nodeIn = nodeOut;
        newEdgeReverse->omega = -omega;
        newEdgeReverse->weight = weight;
        nodeIn->outEdges.push_back(newEdgeReverse);
        nodeOut->inEdges.push_back(newEdgeReverse);
        newEdgeReverse->forward = false;

        edges.push_back(newEdgeReverse);
    }

    vector<Node *> Graph::getNeighborNodes(VectorXd position, double radius)
    {

        vector<Node *> nNodes;
        for (int i = 0; i < nodes.size(); i++)
            if ((position - nodes[i]->position).norm() <= radius)
                nNodes.push_back(nodes[i]);

        return nNodes;
    }

    Node *Graph::getNearestNode(VectorXd position)
    {
        return getNearestNodeList(position)[0];
    }

    vector<Node *> Graph::getNearestNodeList(VectorXd position)
    {
        vector<double> distUnsorted;
        for (int i = 0; i < nodes.size(); i++)
            distUnsorted.push_back((position - nodes[i]->position).norm());

        vector<int> ind = sortGiveIndex(distUnsorted);

        vector<Node *> nodesSorted;
        for (int i = 0; i < ind.size(); i++)
            nodesSorted.push_back(nodes[ind[i]]);

        return nodesSorted;
    }

    double computeDistPath(vector<Edge *> edges)
    {
        double d = 0;
        for (int i = 0; i < edges.size(); i++)
            d += edges[i]->weight;

        return d;
    }


    vector<Edge *> Graph::getPath(Node *origin, Node *target)
    {
        vector<double> V;
        vector<int> edgeToTake;

        for (int i = 0; i < nodes.size(); i++)
        {
            if (i == target->id)
                V.push_back(0.0);
            else
                V.push_back(VERYBIGNUMBER);
            edgeToTake.push_back(-1);
        }

        vector<double> Vold;

        for (int i = 0; i < V.size(); i++)
            Vold.push_back(V[i]);

        bool cont = true;

        do
        {
            for (int i = 0; i < nodes.size(); i++)
            {
                for (int j = 0; j < nodes[i]->outEdges.size(); j++)
                {
                    int idOut = nodes[i]->outEdges[j]->nodeIn->id;
                    double w = nodes[i]->outEdges[j]->weight;

                    if (w + Vold[idOut] < V[i])
                    {
                        V[i] = w + Vold[idOut];
                        edgeToTake[i] = nodes[i]->outEdges[j]->id;
                    }
                }
            }

            cont = false;
            for (int i = 0; i < Vold.size(); i++)
            {
                cont = cont || (V[i] < Vold[i]);
                Vold[i] = V[i];
            }

        } while (cont);

        vector<Edge *> path = {};

        if (V[origin->id] < VERYBIGNUMBER)
        {
            // Its possible!

            // Add self edge

            for (int i = 0; i < edges.size(); i++)
                if ((edges[i]->nodeIn->id == origin->id) && (edges[i]->nodeOut->id == origin->id))
                    path.push_back(edges[i]);

            int currentNode = origin->id;
            int currentEdgeIndex;

            while (currentNode != target->id)
            {
                currentEdgeIndex = edgeToTake[currentNode];
                path.push_back(edges[currentEdgeIndex]);
                currentNode = edges[currentEdgeIndex]->nodeIn->id;
            }
        }

        return path;
    }
}
