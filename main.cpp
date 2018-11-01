#include <QCoreApplication>
#include "dubinspathplanning.h"
#include "rrt.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <stdlib.h>
#include <iostream>
#include <assert.h>

void mapTransformation();

// Инициализация параметров карты и пути
void mapMessageInitParams();
void pathMessageInitParams();

vector<pair<int, int>> resDirection;


// Текущий скан
sensor_msgs::LaserScan current_scan;
// Сообщение с путем
nav_msgs::Path pathMessage;
// Точка для загрузки в сообщение пути
geometry_msgs::PoseStamped point;
// Сообщение с картой

vector<geometry_msgs::Point> obstacleList;

const float ROBOT_WIDTH_HALF = 0.5;
vector<int> gMap;

// Размер карты
float mapResolution = 0.04;
int mapSize = 20/0.04;

int main(int argc, char **argv){
    ros::init(argc, argv, "path_searcher_node");

    // Создание публикатора пути
    ros::NodeHandle l;
    ros::Publisher path_pub = l.advertise<nav_msgs::Path>("/path", 8);
    ros::Publisher map_pub = l.advertise<nav_msgs::OccupancyGrid>("/mapPassability", 2);

    nav_msgs::OccupancyGrid mapMessage;

    mapMessage.info.height = mapSize;
    mapMessage.info.width = mapSize;
    mapMessage.info.resolution = mapResolution;
    mapMessage.info.origin.position.x = -mapSize * mapResolution/2;
    mapMessage.info.origin.position.y = -mapSize * mapResolution/2;
    mapMessage.header.frame_id = "/map";
    mapMessage.header.stamp = ros::Time::now();
    mapMessage.data.resize(mapMessage.info.height * mapMessage.info.width);
    //    mapMessageInitParams();
    pathMessageInitParams();

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
    start.x = mapSize*mapResolution/2 - 5;
    start.y = mapSize*mapResolution/2 - 3;
    start.z = 0.785398;
    geometry_msgs::Point goal;
    goal.x = mapSize*mapResolution/2 - 7;
    goal.y = mapSize*mapResolution/2 - 6;
    goal.z = 0.785398;

    //        DubinsPathPlanning pl;
    //        DubinsPathPlanning::originPath path = pl.dubins_path_planning(0, 0, 0, -5, -7, 0, 1);

    RRT rrt;
    vector<geometry_msgs::Point> path = rrt.Planning(start, goal, obstacleList, 3, 20);

    for (int k = path.size() - 1; k >= 0; k--){
        float nx = path[k].x;
        float ny = path[k].y;
        //        float nx = path.x[k];
        //        float ny = path.y[k];

        // Формируем путь
        point.pose.position.x = nx - mapSize/2*mapResolution;
        point.pose.position.y = ny - mapSize/2*mapResolution;
        cout  <<  point.pose.position.x  << " " <<  point.pose.position.y
               << " " << path[k].z<< endl;

        pathMessage.poses.push_back(point);
    }


    mapTransformation();
        for(int i = 0; i < mapSize*mapSize; i++){
            mapMessage.data[i] = gMap[i];
        }
        for(int i = 0; i < obstacleList.size(); i++){
            geometry_msgs::Point p0 = obstacleList.at(i);
            mapMessage.data[mapSize * (p0.y/mapResolution ) + (p0.x/mapResolution)] = 100;
        }
    //    while(1){
    map_pub.publish(mapMessage);
    path_pub.publish(pathMessage);

    ros::spinOnce();
    //    }


}
void mapMessageInitParams(){

}
void pathMessageInitParams(){
    pathMessage.header.frame_id = "/map";
    pathMessage.header.stamp = ros::Time::now();
}

void mapTransformation(){

    for(int i = 0; i < mapSize*mapSize; i++){
        gMap.push_back(0);
    }

    for(int i = 0; i < obstacleList.size(); i++){
        geometry_msgs::Point p0 = obstacleList.at(i);
        gMap[mapSize * (p0.y/mapResolution ) + (p0.x/mapResolution)] = 100;

        int x0 = p0.x/mapResolution;
        int y0 = p0.y/mapResolution;
        for(int p = x0 - int(ROBOT_WIDTH_HALF / mapResolution) - 1; p < x0 + int(ROBOT_WIDTH_HALF/ mapResolution + 1); p++) {
            for(int q = y0 - int(ROBOT_WIDTH_HALF / mapResolution - 1); q < y0 + int(ROBOT_WIDTH_HALF / mapResolution + 1); q++) {
                float dist =  sqrt(pow((p - x0), 2) + pow((q - y0), 2))*mapResolution;
                if(dist > ROBOT_WIDTH_HALF)
                    continue;

                gMap[mapSize * q + p] = 100;
            }
        }
    }
}
