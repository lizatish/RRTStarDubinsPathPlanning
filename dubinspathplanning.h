#ifndef DUBINSPATHPLANNING_H
#define DUBINSPATHPLANNING_H

#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

class DubinsPathPlanning
{
private:
    struct value{
        float t;
        float p;
        float q;
        string mode;
    };

    struct coords{
        vector<float> x;
        vector<float> y;
        vector<float> yaw;
    };
public:
    struct originPath{
        vector<float> x;
        vector<float> y;
        vector<float> yaw;
        string mode;
        float cost;
    };
    DubinsPathPlanning();

    float mod2pi(float theta);
    float pi_2_pi(float angle);

    value LSL(float alpha, float beta, float d);
    value RSR(float alpha, float beta, float d);
    value LSR(float alpha, float beta, float d);
    value RSL(float alpha, float beta, float d);
    value RLR(float alpha, float beta, float d);
    value LRL(float alpha, float beta, float d);

    originPath dubins_path_planning_from_origin(float ex, float ey, float feyaw, float c);
    originPath dubins_path_planning(float sx, float sy, float syaw,
                                    float ex, float ey, float eyaw, float c);

    coords generate_course(float* length, string mode, float c);
};

#endif // DUBINSPATHPLANNING_H
