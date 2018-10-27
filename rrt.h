#ifndef RRT_H
#define RRT_H

#include <ros/ros.h>
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

    float minDistToObstacle = 0.04;
    vector<Node*> nodeList;
    float curvature;

    vector<geometry_msgs::Point> resultPath;

public:
    RRT();
    void Planning(geometry_msgs::Point start, geometry_msgs::Point goal,
                  vector<float> map, float curv, float map_size,int maxItem = 100);

    vector<geometry_msgs::Point> formObstaclesCoordinatesFromMap(vector<float> map, int mapSize);
    Node* getRandomPoint();
    int getNearestListIndex(Node* rnd);

    float pi_2_pi(float angle);
    Node* steer(Node* rnd, int nind);
    bool collisionCheck(Node* node);
    vector<geometry_msgs::Point> gen_final_course(int goalInd);
    float get_best_last_index();
    Node* choose_parent(Node* newNode, vector<int> nearInds);


};

#endif // RRT_H
