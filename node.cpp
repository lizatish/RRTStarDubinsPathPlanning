#include "node.h"
Node::Node(){
    x = 0;
    y = 0;
    yaw = 0;
    cost = 0.0;
    parent = NAN;
}
Node::Node(float x0, float y0, float yaw0)
{
    x = x0;
    y = y0;
    yaw = yaw0;
    cost = 0.0;
    parent = -1;
}
