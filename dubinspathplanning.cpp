#include "dubinspathplanning.h"

DubinsPathPlanning::DubinsPathPlanning()
{

}

float DubinsPathPlanning::mod2pi(float theta){
    return theta - 2.0 * M_PI * floor(theta / 2.0 / M_PI);
}


float DubinsPathPlanning::pi_2_pi(float angle){
    if(angle < -M_PI)
        return M_PI - abs(fmod((angle + M_PI), (2 * M_PI)));
    else if(angle < 0)
        return abs(fmod((angle + M_PI), (2 * M_PI))) - M_PI;
    else
        return  fmod((angle + M_PI), (2 * M_PI)) - M_PI;
}


DubinsPathPlanning::value DubinsPathPlanning::LSL(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    float tmp0 = d + sa - sb;

    value v1;
    v1.mode = "LSL";

    float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if (p_squared < 0){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }

    float tmp1 = atan2((cb - ca), tmp0);
    v1.t = mod2pi(-alpha + tmp1);
    v1.p = sqrt(p_squared);
    v1.q = mod2pi(beta - tmp1);

    return v1;
}

DubinsPathPlanning::value DubinsPathPlanning::RSR(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    value v1;
    float tmp0 = d - sa + sb;
    v1.mode = "RSR";
    float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
    if (p_squared < 0){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }
    float tmp1 = atan2((ca - cb), tmp0);
    v1.t = mod2pi(alpha - tmp1);
    v1.p = sqrt(p_squared);
    v1.q = mod2pi(-beta + tmp1);

    return v1;
}

DubinsPathPlanning::value DubinsPathPlanning::LSR(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    value v1;
    v1.mode = "LSR";
    float p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
    if (p_squared < 0){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }

    v1.p = sqrt(p_squared);
    float tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, v1.p);
    v1.t = mod2pi(-alpha + tmp2);
    v1.q = mod2pi(-mod2pi(beta) + tmp2);

    return v1;
}

DubinsPathPlanning::value DubinsPathPlanning::RSL(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    value v1;
    float p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
    v1.mode = "RSL";
    if (p_squared < 0){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }
    v1.p = sqrt(p_squared);
    float tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, v1.p);
    v1.t = mod2pi(alpha - tmp2);
    v1.q = mod2pi(beta - tmp2);

    return v1;
}

DubinsPathPlanning::value DubinsPathPlanning::RLR(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    value v1;
    v1.mode = "RLR";
    float tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    if (abs(tmp_rlr) > 1.0){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }

    v1.p = mod2pi(2 * M_PI - acos(tmp_rlr));
    v1.t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(v1.p / 2.0));
    v1.q = mod2pi(alpha - beta - v1.t + mod2pi(v1.p));
    return v1;
}

DubinsPathPlanning::value DubinsPathPlanning::LRL(float alpha, float beta, float d){
    float sa = sin(alpha);
    float sb = sin(beta);
    float ca = cos(alpha);
    float cb = cos(beta);
    float c_ab = cos(alpha - beta);

    value v1;
    v1.mode = "LRL";
    float tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.;
    if (abs(tmp_lrl) > 1){
        v1.p = NAN;
        v1.q = NAN;
        v1.t = NAN;
        return v1;
    }
    v1.p = mod2pi(2 * M_PI - acos(tmp_lrl));
    v1.t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + v1.p / 2.);
    v1.q = mod2pi(mod2pi(beta) - alpha - v1.t + mod2pi(v1.p));

    return v1;
}


DubinsPathPlanning::originPath DubinsPathPlanning::dubins_path_planning_from_origin(float ex, float ey, float eyaw, float c){
    float dx = ex;
    float dy = ey;
    float D = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    float d = D / c;

    float theta = mod2pi(atan2(dy, dx));
    float alpha = mod2pi(- theta);
    float beta = mod2pi(eyaw - theta);
    float bcost = INFINITY;
    value b;
    b.t = NAN;
    b.p = NAN;
    b.q = NAN;
    b.mode = NAN;

    value v1;

    typedef DubinsPathPlanning::value (DubinsPathPlanning::*PA)(float alpha, float beta, float d);
    PA planner[] = {&DubinsPathPlanning::LSL,
                    &DubinsPathPlanning::RSR,
                    &DubinsPathPlanning::LSR,
                    &DubinsPathPlanning::RSL,
                    &DubinsPathPlanning::RLR,
                    &DubinsPathPlanning::LRL};

    for (int i = 0; i < 6; i++){
        v1 = (this->*planner[i])(alpha, beta, d);

        if (isnan(v1.t)){
            continue;
        }

        float cost = (abs(v1.t) + abs(v1.p) + abs(v1.q));
        if (bcost > cost){
            b.t = v1.t;
            b.p = v1.p;
            b.q = v1.q;
            b.mode = v1.mode;
            bcost = cost;
        }
    }

    coords cds;
    float bMass[3] = {b.t, b.p, b.q};

    cds = generate_course(bMass, b.mode, c);

    originPath origin;
    origin.x = cds.x;
    origin.y = cds.y;
    origin.yaw = cds.yaw;
    origin.cost = bcost;
    origin.mode = b.mode;

    return origin;
}
DubinsPathPlanning::originPath DubinsPathPlanning::dubins_path_planning(float startX, float startY, float startYaw,
                                                                        float exitX, float exitY, float exitYaw,
                                                                        float curvature){

    exitX = exitX - startX;
    exitY = exitY - startY;

    float lex = cos(startYaw) * exitX + sin(startYaw) * exitY;
    float ley = - sin(startYaw) * exitX + cos(startYaw) * exitY;
    float leyaw = exitYaw - startYaw;

    originPath orig;
    orig = dubins_path_planning_from_origin(lex, ley, leyaw, curvature);
    vector<float> lpx = orig.x;
    vector<float> lpy = orig.y;
    vector<float> lpyaw = orig.yaw;
    string mode = orig.mode;
    float clen = orig.cost;

    vector<float> px, py;
    for(int i = 0; i < lpx.size(); i++){
        px.push_back(cos(-startYaw) * lpx[i] + sin(-startYaw) * lpy[i] + startX);
        py.push_back( -sin(-startYaw) * lpx[i] + cos(-startYaw) * lpy[i] + startY);
    }

    vector<float> pyaw;
    for(int i = 0; i < lpyaw.size(); i++){
        pyaw.push_back(pi_2_pi(lpyaw[i] + startYaw));
    }

    originPath res;
    res.x = px;
    res.y = py;
    res.yaw = pyaw;
    res.mode = mode;
    res.cost = clen;
    return res;
}

DubinsPathPlanning::coords DubinsPathPlanning::generate_course(float* length, string mode, float c){

    vector<float> px = {0.0};
    vector<float> py = {0.0};
    vector<float> pyaw = {0.0};
    for (int i = 0; i < mode.length(); i++){

        float pd = 0.0;
        float d;
        if (mode[i] == 'S'){
            d = 1.0 / c;
        }
        else{
            d = 3.0 * M_PI / 180;
        }

        while (pd < abs(length[i] - d)){
            px.push_back(px[px.size()-1] + d * c * cos(pyaw[pyaw.size()-1]));
            py.push_back(py[py.size()-1] + d * c * sin(pyaw[pyaw.size()-1]));

            if (mode[i] == 'L'){
                pyaw.push_back(pyaw[pyaw.size()-1] + d);
            }
            else if (mode[i] == 'S'){
                pyaw.push_back(pyaw[pyaw.size()-1]);
            }
            else if (mode[i] == 'R'){
                pyaw.push_back(pyaw[pyaw.size()-1] - d);
            }
            pd += d;
        }
        d = length[i] - pd;
        px.push_back(px[px.size()-1] + d * c * cos(pyaw[pyaw.size()-1]));
        py.push_back(py[py.size()-1] + d * c * sin(pyaw[pyaw.size()-1]));

        if (mode[i] == 'L'){
            pyaw.push_back(pyaw[pyaw.size()-1] + d);
        }
        else if (mode[i] == 'S'){
            pyaw.push_back(pyaw[pyaw.size()-1]);
        }
        else if (mode[i] == 'R'){
            pyaw.push_back(pyaw[pyaw.size()-1] - d);
        }
        pd += d;
    }

    coords p;
    p.x = px;
    p.y = py;
    p.yaw = pyaw;
    return p;
}
