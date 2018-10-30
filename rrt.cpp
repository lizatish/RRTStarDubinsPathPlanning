#include "rrt.h"

RRT::RRT(){
}
vector<geometry_msgs::Point> RRT::Planning(geometry_msgs::Point s, geometry_msgs::Point g,
                                           vector<geometry_msgs::Point> ob ,/*vector<float> map0,*/ float curv, float mapSize0,int maxIter0)
{

    start = new Node(s.x, s.y, s.z);
    end = new Node(g.x, g.y, g.z);
    mapSize = mapSize0;
    minRand = 0;
    maxRand = mapSize0;
    goalSampleRate = 10;
    maxIter = 100;
    //    vector<float> map = map0;
    curvature = curv;

    obstacleList =  ob ;
    // Найти массив координат препятствий
    //formObstaclesCoordinatesFromMap(map, mapSize);

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
        //        if (animation and i % 5 == 0)
        //            self.DrawGraph(rnd=rnd);
    }
    int lastIndex = get_best_last_index();

    vector<geometry_msgs::Point> path = gen_final_course(lastIndex);
    return path;
}
void RRT::formObstaclesCoordinatesFromMap(vector<float> map, int mapSize){

    for(int i = 0; i < mapSize; i++){
        for(int j = 0; j < mapSize; j++){
            if(map[mapSize * j + i] == 100){
                geometry_msgs::Point p;
                p.x = i;
                p.y = j;
                obstacleList.push_back(p);
            }
        }
    }
}

Node* RRT::choose_parent(Node* newNode, vector<int> nearInds){
    if (nearInds.size() == 0){
        return newNode;
    }

    vector<float> dlist;
    for(int i = 0; i < nearInds.size(); i++){
        Node* tNode = steer(newNode, i);
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

    Node* newNode = new Node();

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
        if(nodeList[fgoalinds[i]]->cost != 0)
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
    path.push_back(p);

    while(nodeList[goalInd]->parent != -1){
        Node* node = nodeList[goalInd];

        for(int i = node->path_x.size() - 1; i >= 0; i--){
            p.x = node->path_x[i];
            p.y = node->path_y[i];
            path.push_back(p);
        }
        goalInd = node->parent;
    }
    p.x = start->x;
    p.y = start->y;
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
        Node* nearNode = nodeList[i];
        Node* tNode = steer(nearNode, nnode - 1);

        bool obstacleOK = collisionCheck(tNode);
        bool imporveCost = nearNode->cost > tNode->cost;

        if (obstacleOK && imporveCost){
            nodeList[i] = tNode;
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

            d = dx * dx + dy * dy;
            if (d <= pow(minDistToObstacle, 2))
                return false;
        }
    }
    return true;
}
