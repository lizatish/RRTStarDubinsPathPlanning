#include "rrt.h"

RRT::RRT(){
}
vector<geometry_msgs::Point> RRT::Planning(geometry_msgs::Point s, geometry_msgs::Point g,
                                           vector<geometry_msgs::Point> ob , float curv,
                                           float robot_width_half,float mapSize0,
                                           float mapRes,int maxIter0)
{
    mapSize = mapSize0;
    mapResolution = mapRes;
    minRand = 0;
    maxRand = mapSize0*mapResolution;
    // РАЗОБРАТЬСЯ С ЭТИМ ПАРАМЕТРОМ
    goalSampleRate = 10;
    maxIter = maxIter0;
    curvature = curv;
    minDistToObstacle = robot_width_half + mapResolution;

    start = new Node(metrs2cells(s.x), metrs2cells(s.y), s.z);
    end = new Node(metrs2cells(g.x), metrs2cells(g.y), g.z);

    for(int i = 0; i < ob.size(); i++){
        geometry_msgs::Point p = ob.at(i);
        geometry_msgs::Point k;

        k.x = metrs2cells(p.x);
        k.y = metrs2cells(p.y);
        obstacleList.push_back(k);
    }

    nodeList.push_back(start);

    for(int i = 0; i < maxIter; i++){

        Node* rnd = getRandomPoint();
        int nind = getNearestListIndex(rnd);

        Node* newNode = steer(rnd, nind);

        if (collisionCheck(newNode)){
            vector<int> nearInds = find_near_nodes(newNode);
            newNode = choose_parent(newNode, nearInds);
            nodeList.push_back(newNode);
            rewire(nearInds);
        }
    }
    int lastIndex = get_best_last_index();

    vector<geometry_msgs::Point> path = gen_final_course(lastIndex);
    return path;
}
float RRT::metrs2cells(float metrs){
    return (mapSize/2*mapResolution + metrs);
}

Node* RRT::choose_parent(Node* newNode, vector<int> nearInds){
    if (nearInds.size() == 0){
        return newNode;
    }

    vector<float> dlist;
    for(int i = 0; i < nearInds.size(); i++){
        Node* tNode = steer(newNode, nearInds[i]);
        if (collisionCheck(tNode)){
            dlist.push_back(tNode->cost);
        }
        else{
            dlist.push_back(INFINITY);
        }
    }

    float mincost = *min_element(dlist.begin(), dlist.end());
    vector<float>::iterator it = find(dlist.begin(), dlist.end(), mincost);
    int minind = nearInds[distance(dlist.begin(), it)];


    if (mincost == INFINITY){
        return newNode;
    }

    newNode = steer(newNode, minind);

    return newNode;
}
float RRT::pi_2_pi(float angle){
    return fmod((angle + M_PI), (2 * M_PI) - M_PI);
}
Node* RRT::steer(Node* rnd, int nind){

    Node* nearestNode = nodeList[nind];

    DubinsPathPlanning* DP;

    DubinsPathPlanning::originPath path;
    path = DP->dubins_path_planning(
                nearestNode->x, nearestNode->y, nearestNode->yaw,
                rnd->x, rnd->y, rnd->yaw,
                curvature);

    Node* newNode = new Node;

    if(path.yaw.size() > 0){
        newNode->yaw = path.yaw[path.yaw.size()-1];
        newNode->path_yaw = path.yaw;
    }
    if(path.x.size() > 0){
        newNode->x = path.x[path.x.size()-1];
        newNode->y = path.y[path.y.size()-1];

        newNode->path_x = path.x;
        newNode->path_y = path.y;
    }
    newNode->cost = nearestNode->cost + path.cost;
    newNode->parent = nind;

    return newNode;
}
Node* RRT::getRandomPoint(){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> random(0, 100);
    std::uniform_real_distribution<> random_xy(minRand, maxRand);
    std::uniform_real_distribution<> random_yaw(-M_PI, M_PI);

    geometry_msgs::Point rnd;
    if (random(gen) > goalSampleRate){
        rnd.x = random_xy(gen);
        rnd.y = random_xy(gen);
        rnd.z = random_yaw(gen);
    }
    else{
        rnd.x = end->x;
        rnd.y = end->y;
        rnd.z = end->yaw;
    }

    Node* node = new Node(rnd.x, rnd.y, rnd.z);
    return node;
}
float RRT::get_best_last_index(){

    const float YAWTH = 1 * M_PI / 180;
    const float XYTH = 0.5;

    vector<int> goalinds;
    for (int k = 0; k < nodeList.size(); k++){
        Node* node = nodeList[k];
        if(calc_dist_to_goal(node->x, node->y) <= XYTH){
            goalinds.push_back(k);
        }
    }

    vector<int> fgoalinds;
    for(int i = 0; i < goalinds.size(); i++){
        if (abs(nodeList[goalinds[i]]->yaw - end->yaw) <= YAWTH){
            fgoalinds.push_back(goalinds[i]);
        }
    }

    if (fgoalinds.size() == 0){
        return NAN;
    }

    vector<float> cost;
    for(int i = 0; i < fgoalinds.size(); i++){
        cost.push_back(nodeList[fgoalinds[i]]->cost);
    }
    float mincost = *min_element(cost.begin(), cost.end());

    for(int i = 0; i < fgoalinds.size(); i++){
        if (nodeList[fgoalinds[i]]->cost == mincost){
            return fgoalinds[i];
        }
    }

    return NAN;
}
vector<geometry_msgs::Point> RRT::gen_final_course(int goalInd){
    vector<geometry_msgs::Point> path;
    geometry_msgs::Point p;
    p.x = end->x;
    p.y = end->y;
    p.z = end->yaw;
    path.push_back(p);

    if(goalInd < -1)
        return path;

    while(nodeList[goalInd]->parent != -1){
        Node* node = nodeList[goalInd];

        for(int i = node->path_x.size() - 1; i >= 0; i--){
            p.x = node->path_x[i];
            p.y = node->path_y[i];
            p.z = node->path_yaw[i];
            path.push_back(p);
        }
        goalInd = node->parent;
    }
    p.x = start->x;
    p.y = start->y;
    p.z = start->yaw;
    path.push_back(p);
    return path;
}

float RRT::calc_dist_to_goal(float x, float y){
    return sqrt(pow(x - end->x, 2) + pow(y - end->y, 2));
}

vector<int> RRT::find_near_nodes(Node* newNode){
    int nnode = nodeList.size();
    float r = 50.0 * sqrt((log(nnode) / nnode));

    vector<float> dlist;
    for(int i = 0; i < nodeList.size(); i++){
        float k = pow(nodeList[i]->x - newNode->x, 2) +
                pow(nodeList[i]->y - newNode->y, 2) +
                pow(nodeList[i]->yaw - newNode->yaw, 2);

        dlist.push_back(k);
    }

    vector<int> nearinds;
    for(int i = 0; i < dlist.size(); i++){
        if(dlist[i] <= r*r){
            nearinds.push_back(i);
        }
    }
    return nearinds;
}

void RRT::rewire(vector<int> nearinds){

    int nnode = nodeList.size();
    for(int i = 0; i < nearinds.size(); i++){
        Node* nearNode = nodeList[nearinds[i]];
        Node* tNode = steer(nearNode, nnode - 1);

        bool obstacleOK = collisionCheck(tNode);
        bool imporveCost = nearNode->cost > tNode->cost;

        if (obstacleOK && imporveCost){
            nodeList[nearinds[i]] = tNode;
        }
    }
}

int RRT::getNearestListIndex(Node* rnd){

    vector<float> dlist;
    for(int i = 0; i < nodeList.size(); i++){
        float n = pow(nodeList[i]->x - rnd->x, 2) +
                pow(nodeList[i]->y - rnd->y, 2) +
                pow(nodeList[i]->yaw - rnd->yaw, 2);
        dlist.push_back(n);
    }
    auto min_value = *min_element(dlist.begin(), dlist.end());
    vector<float>::iterator it = find(dlist.begin(), dlist.end(), min_value);
    int minIndex = distance(dlist.begin(), it);

    return minIndex;
}

// Проверка на пересечение с препятствиями
bool RRT::collisionCheck(Node* node){

    float ix, iy, dx, dy, d;
    geometry_msgs::Point p;
    for (int i = 0; i < obstacleList.size(); i++){

        p = obstacleList.at(i);
        for(int j = 0; j < node->path_x.size(); j++){

            ix = node->path_x[j];
            iy = node->path_y[j];
            dx = p.x - ix;
            dy = p.y - iy;

           d = pow(dx, 2) + pow(dy, 2);

            if (d <= pow(minDistToObstacle,2))
                return false;
        }
    }
    return true;
}
