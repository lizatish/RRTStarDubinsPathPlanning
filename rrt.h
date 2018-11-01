#ifndef RRT_H
#define RRT_H

#include <geometry_msgs/Point.h>

#include <math.h>
#include <vector>
#include <iostream>

#include <random>

#include "node.h"
#include "dubinspathplanning.h"

using namespace std;

class RRT
{
private:
    vector<float> map;
    vector<geometry_msgs::Point> obstacleList;
    float mapSize;

    int minRand;
    int maxRand;
    Node* start;
    Node* end;
    int goalSampleRate;
    int maxIter;

    float minDistToObstacle = 0.5 + 0.04;
    vector<Node*> nodeList;
    float curvature;

    vector<geometry_msgs::Point> resultPath;

public:
    RRT();

    vector<geometry_msgs::Point> Planning(geometry_msgs::Point s, geometry_msgs::Point g,
                     vector<geometry_msgs::Point> ob /* vector<float> map0*/, float curv, float mapSize0,int maxIter0 = 20);

    void formObstaclesCoordinatesFromMap(vector<float> map, int mapSize);
    Node* getRandomPoint();
    int getNearestListIndex(Node* rnd);

    float pi_2_pi(float angle);
    Node* steer(Node* rnd, int nind);
    bool collisionCheck(Node* node);
    vector<geometry_msgs::Point> gen_final_course(int goalInd);
    float get_best_last_index();
    Node* choose_parent(Node* newNode, vector<int> nearInds);
    void rewire(vector<int> nearinds);

    vector<int> find_near_nodes(Node* newNode);

    float calc_dist_to_goal(float x, float y);



};

#endif // RRT_H
