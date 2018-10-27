#ifndef DUBINSPATHPLANNING_H
#define DUBINSPATHPLANNING_H

#include <math.h>
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
        vector<float> px;
        vector<float> py;
        vector<float> pyaw;
    };
public:
    struct originPath{
        vector<float> px;
        vector<float> py;
        vector<float> pyaw;
        string bmode;
        float bcost;
    };
    DubinsPathPlanning();

    float mod2pi(float theta);
    int pi_2_pi(float angle);

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
    //void plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k");

    int **zip(int *arr1, string arr2, int length);

};

#endif // DUBINSPATHPLANNING_H
