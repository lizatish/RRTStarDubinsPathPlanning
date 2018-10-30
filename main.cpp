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


// Текущая локальная карта
vector<int> localMap;

// Текущая открытая глобальная карта
vector<int> globalMap;

// Размер карты
float mapResolution = 0.04;
int mapSize = 15/0.04;
int main(int argc, char **argv){
    // Сообщение с картой
    nav_msgs::OccupancyGrid mapMessage;

    ros::init(argc, argv, "path_searcher_node");


    // Создание публикатора пути
    ros::NodeHandle l;
    ros::Publisher path_pub = l.advertise<nav_msgs::Path>("/path", 8);

    // Создание публикатора карты
    ros::NodeHandle m;
    ros::Publisher map_pub = m.advertise<nav_msgs::OccupancyGrid>("/mapPassability", 2);

    mapMessage.info.height = mapSize;
    mapMessage.info.width = mapSize;
    mapMessage.info.resolution = mapResolution;
    mapMessage.info.origin.position.x = -mapSize * mapResolution/2;
    mapMessage.info.origin.position.y = -mapSize * mapResolution/2;
    mapMessage.header.frame_id = "/map";
    mapMessage.header.stamp = ros::Time::now();
    mapMessage.data.resize(mapMessage.info.height * mapMessage.info.width);

    pathMessageInitParams();
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
    start.x = mapSize*mapResolution/2;
    start.y = mapSize*mapResolution/2;
    start.z = 0;
    geometry_msgs::Point goal;
    goal.x = mapSize*mapResolution/2 + 6;
    goal.y = mapSize*mapResolution/2 + 6;
    goal.z = 0;




    //    for(int i = 0; i < mapSize*mapSize; i++){
    //        mapMessage.data[i] = 0;
    //    }

    //    srand(time(0));
    //    for(int i = 1; i < mapSize - 1; i++){
    //        for(int j = 1; j < mapSize - 1; j++){
    //            int k = rand()%2;
    //            if(!k){
    //                int x2 = i;
    //                int y2 = j;
    //                mapMessage.data[mapSize * y2 + x2] = 100;
    //                mapMessage.data[mapSize * (y2 + 1) + (x2 + 1)] = 100;
    //                mapMessage.data[mapSize * (y2 + 1) + x2] = 100;
    //                mapMessage.data[mapSize * (y2 + 1) + (x2 - 1)] = 100;
    //                mapMessage.data[mapSize * (y2 - 1) + (x2 + 1)] = 100;
    //                mapMessage.data[mapSize * (y2 - 1) + (x2 - 1)] = 100;
    //                mapMessage.data[mapSize * (y2 - 1) + x2] = 100;
    //                mapMessage.data[mapSize * y2 + (x2 - 1)] = 100;
    //                mapMessage.data[mapSize * y2 + (x2 + 1)] = 100;
    //            }
    //        }
    //    }



    RRT rrt;
    vector<geometry_msgs::Point> path = rrt.Planning(start, goal, obstacleList, 1, 15);

    for (int k = path.size() - 1; k >= 0; k--){
        float nx = path[k].x;
        float ny = path[k].y;

        // Формируем путь
        point.pose.position.x = nx - mapSize/2*mapResolution;
        point.pose.position.y = ny - mapSize/2*mapResolution;
        pathMessage.poses.push_back(point);
    }
    for (int i = 0; i < path.size(); i++){
        geometry_msgs::Point p0 = path.at(i);
        cout  << p0.x  << " " << p0.y << endl;
    }

    for(int i = 0; i < mapSize*mapSize; i++){
        mapMessage.data[i] = 0;
    }
    for(int i = 0; i < obstacleList.size(); i++){
        geometry_msgs::Point p0 = obstacleList.at(i);
        mapMessage.data[mapSize * (p0.y/mapResolution ) + (p0.x/mapResolution)] = 100;
    }

    map_pub.publish(mapMessage);
    path_pub.publish(pathMessage);

    ros::spinOnce();

}
void mapMessageInitParams(){
    //    mapMessage.info.height = mapSize;
    //    mapMessage.info.width = mapSize;
    //    mapMessage.info.resolution = mapResolution;
    //    mapMessage.info.origin.position.x = -mapSize * mapResolution/2;
    //    mapMessage.info.origin.position.y = -mapSize * mapResolution/2;
    //    mapMessage.header.frame_id = "/map";
    //    mapMessage.header.stamp = ros::Time::now();
    //    mapMessage.data.resize(mapMessage.info.height * mapMessage.info.width);
}
void pathMessageInitParams(){
    pathMessage.header.frame_id = "/map";
    pathMessage.header.stamp = ros::Time::now();
}
