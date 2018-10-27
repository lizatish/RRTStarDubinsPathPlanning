#include "rrt.h"

RRT::RRT(){
}
void RRT::Planning(geometry_msgs::Point s, geometry_msgs::Point g,
                   vector<float> map0, float curv, float mapSize0,int maxIter0)
{

    start = new Node(s.x, s.y, s.z);
    end = new Node(g.x, g.y, g.z);
    mapSize = mapSize0;
    minRand = 0;
    maxRand = mapSize0;
    goalSampleRate = 10;
    maxIter = maxIter0;
    vector<float> map = map0;
    curvature = curv;

    // Найти массив координат препятствий
    vector<geometry_msgs::Point> obstacles;
    obstacles = formObstaclesCoordinatesFromMap(map, mapSize);

    nodeList.push_back(start);

    for(int i = 0; i < maxIter; i++){

        Node* rnd = getRandomPoint();
        int nind = getNearestListIndex(rnd);

        Node* newNode = steer(rnd, nind);

        if (collisionCheck(newNode)){
            int nearInds = find_near_nodes(newNode);
            Node* newNode = choose_parent(newNode, nearInds);
            nodeList.push_back(newNode);
            rewire(newNode, nearInds);
        }
        //        if (animation and i % 5 == 0)
        //            self.DrawGraph(rnd=rnd);
    }
    int lastIndex = get_best_last_index();

    if (lastIndex == NAN)
        return NAN;

    path = gen_final_course(lastIndex);
    return path;
}
vector<geometry_msgs::Point> formObstaclesCoordinatesFromMap(vector<float> map, int mapSize){

    vector<geometry_msgs::Point>  obstacles;
    for(int i = 0; i < mapSize; i++){
        for(int j = 0; j < mapSize; j++){
            if(map[mapSize * j + i] == 100){
                geometry_msgs::Point p;
                p.x = i;
                p.y = j;
                obstacles.push_back(p);
            }
        }
    }
    return obstacles;
}

Node* RRT::choose_parent(Node* newNode, vector<int> nearInds){
    if (nearInds.size() == 0){
        return newNode;
    }

    vector<float> dlist;
    for(int i = 0; i < nearInds.size(); i++){
        Node* tNode = steer(newNode, i);
        if (collisionCheck(tNode, obstacleList)){
            dlist.append(tNode.cost);
        }
        else{
            dlist.push_back(INFINITY);
        }
    }

    float mincost = *std::min_element(dlist.begin(), dlist.end());

    auto min_value = *min_element(dlist.begin(), dlist.end());
    vector<Node*>::iterator it = find(dlist.begin(), dlist.end(), min_value);
    int minIndex = distance(dlist.begin(), it);

    int minind = nearInds[dlist.index(mincost)];

    if (mincost == INFINITY){
        return newNode;
    }

    newNode = steer(newNode, minIndex);

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

    Node* newNode;
    newNode->x = path.x[path.x.size()-1];
    newNode->y = path.y[path.y.size()-1];
    newNode->yaw = path.yaw[path.yaw.size()-1];

    newNode->path_x = path.x;
    newNode->path_y = path.y;
    newNode->path_yaw = path.yaw;
    newNode->cost += path.cost;
    newNode->parent = nind;

    return newNode;
}
Node* RRT::getRandomPoint(){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> random(0, 100);
    std::uniform_int_distribution<> random_xy(minRand, maxRand);
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
        if(calc_dist_to_goal(node.x, node.y) <= XYTH){
            goalinds.push_back(k);
        }
    }

    vector<int> fgoalinds;
    for(int i = 0; i < goalinds.size(); i++){
        if (abs(nodeList[i]->yaw - end->yaw) <= YAWTH){
            fgoalinds.push_back(i);
        }
    }

    if (fgoalinds.size() == 0){
        return NAN;
    }

    vector<float> cost;
    for(int i = 0; i < fgoalinds.size(); i++){
        cost.push_back(nodeList[i]->cost);
    }
    float mincost = *std::min_element(cost.begin(), cost.end());

    for(int i = 0; i < fgoalinds.size(); i++){
        if (nodeList[i]->cost == mincost){
            return i;
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

    while (nodeList[goalInd].parent != NAN){
        Node* node = self.nodeList[goalInd];
        for ((ix, iy) in zip(reversed(node->path_x), reversed(node->path_y)):
             path.append([ix, iy])

             goalind = node.parent
             path.append([self.start.x, self.start.y])
             return path
    }

             def calc_dist_to_goal(self, x, y){
             return np.linalg.norm([x - self.end.x, y - self.end.y])
    }

             def find_near_nodes(self, newNode){
             nnode = len(self.nodeList)
             r = 50.0 * math.sqrt((math.log(nnode) / nnode))
     #  r = self.expandDis * 5.0
             dlist = [(node.x - newNode.x) ** 2 +
             (node.y - newNode.y) ** 2 +
             (node.yaw - newNode.yaw) ** 2
             for node in self.nodeList]
             nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
             return nearinds
    }

             def rewire(self, newNode, nearinds){

             nnode = len(self.nodeList)

             for i in nearinds:
             nearNode = self.nodeList[i]
             tNode = self.steer(nearNode, nnode - 1)

             obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
             imporveCost = nearNode.cost > tNode.cost

             if obstacleOK and imporveCost:
             self.nodeList[i] = tNode
    }

             int RRT::getNearestListIndex(Node* rnd){

             vector<Node*> dlist;
             for(int i = 0; i < nodeList.size(); i++){
             Node* n = new Node(pow(nodeList[i]->x - rnd->x, 2),
                                pow(nodeList[i]->y - rnd->y, 2),
                                pow(nodeList[i]->yaw - rnd->yaw, 2));
             dlist.push_back(n);
    }
             auto min_value = *min_element(dlist.begin(), dlist.end());
             vector<Node*>::iterator it = find(dlist.begin(), dlist.end(), min_value);
             int minIndex = distance(dlist.begin(), it);

             return minIndex;
    }
             bool RRT::collisionCheck(Node* node){

             float ix, iy, dx, dy, d;
             geometry_msgs::Point p;
             for (int i = 0; i < obstacleList.size(); i++){

             p = obstacleList.at(i);

             ix = node->path_x[i];
             iy = node->path_y[i];
             dx = p.x - ix;
             dy = p.y - iy;

             d = dx * dx + dy * dy;
             if (d <= pow(minDistToObstacle, 2))
             return false;
    }
             return true;
    }
             //            def DrawGraph(self, rnd=None){
             //                plt.clf()
             //                if rnd is not None:
             //                plt.plot(rnd.x, rnd.y, "^k")
             //                for node in self.nodeList:
             //                if node.parent is not None:
             //                plt.plot(node.path_x, node.path_y, "-g")
             //    #  plt.plot([node.x, self.nodeList[node.parent].x], [
             //    #  node.y, self.nodeList[node.parent].y], "-g")

             //                for (ox, oy, size) in self.obstacleList:
             //                plt.plot(ox, oy, "ok", ms=30 * size)

             //                dubins_path_planning.plot_arrow(
             //                self.start.x, self.start.y, self.start.yaw)
             //                dubins_path_planning.plot_arrow(
             //                self.end.x, self.end.y, self.end.yaw)

             //                plt.axis([-2, 15, -2, 15])
             //                plt.grid(True)
             //                plt.pause(0.01)
             //            }
