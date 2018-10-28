#include <QCoreApplication>
#include "dubinspathplanning.h"
#include "rrt.h"

int main(int argc, char *argv[])
{
    vector<geometry_msgs::Point> obstacleList;
    geometry_msgs::Point p;
    p.x = 5;
    p.y = 5;
    obstacleList.push_back(p);
    p.x = 3;
    p.y = 6;
    obstacleList.push_back(p);
    p.x = 3;
    p.y = 8;
    obstacleList.push_back(p);
    p.x = 3;
    p.y = 10;
    obstacleList.push_back(p);
    p.x = 7;
    p.y = 5;
    obstacleList.push_back(p);
    p.x = 9;
    p.y = 5;
    obstacleList.push_back(p);

    geometry_msgs::Point start;
    start.x = 0;
    start.y = 0;
    start.z = 2*0.785398;
    geometry_msgs::Point goal;
    goal.x = 12;
    goal.y = 12;
    goal.z = 2*0.785398;

    RRT rrt;
    vector<geometry_msgs::Point> path = rrt.Planning(start, goal, obstacleList, 1, 15);

    //    DubinsPathPlanning p;
    //    DubinsPathPlanning::originPath res = p.dubins_path_planning(1, 1, 45, 10, 10, 90, 1);
      for (int i = 0; i < path.size(); i++){
          geometry_msgs::Point p0 = path.at(i);
           cout  << p0.x  << " " << p0.y << endl;
      }
}
