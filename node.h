#ifndef NODE_H
#define NODE_H

#include <math.h>
#include <vector>
#include <iostream>
using namespace std;
#include <limits>

class Node
{
public:
    float x;
    float y;
    float yaw;

    vector<float> path_x;
    vector<float> path_y;
    vector<float> path_yaw;

    float cost;
    int parent;

    Node();
    Node(float x0, float y0, float yaw0);
//    float getX() const;
//    void setX(float value);
};

#endif // NODE_H
