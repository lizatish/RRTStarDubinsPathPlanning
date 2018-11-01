#include "rrt.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

void drawCircleObstacles(float radius);

// Инициализация параметров карты и пути
void mapMessageInitParams();
void pathMessageInitParams();

nav_msgs::OccupancyGrid mapMessage;
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
    ros::Publisher map_pub = l.advertise<nav_msgs::OccupancyGrid>("/gMap", 2);

    mapMessageInitParams();
    pathMessageInitParams();

    for(int i = 0; i < mapSize*mapSize; i++){
        gMap.push_back(0);
    }

    geometry_msgs::Point p;

    p.x = 5;
    p.y = 5;
    obstacleList.push_back(p);
    p.x = 3;
    p.y = 3;
    obstacleList.push_back(p);
    p.x = 3;
    p.y = 3;
    obstacleList.push_back(p);
    p.x = 7;
    p.y = 5;
    obstacleList.push_back(p);
    p.x = 6;
    p.y = 5;
    obstacleList.push_back(p);
    p.x = 1;
    p.y = 2;
    obstacleList.push_back(p);
    p.x = 0;
    p.y = 3;
    obstacleList.push_back(p);
    p.x = 2;
    p.y = 6;
    obstacleList.push_back(p);
    p.x = 2;
    p.y = 5;
    obstacleList.push_back(p);


    // Координаты старта и финиша
    geometry_msgs::Point start;
    start.x = 0;
    start.y = 0;
    start.z = 0.785398;
    geometry_msgs::Point goal;
    goal.x = 4;
    goal.y = 8;
    goal.z = -4*0.78;

    // Запуск планировщика
    RRT rrt;
    vector<geometry_msgs::Point> path = rrt.Planning(start, goal, obstacleList, 1, mapSize);

    // Отрисовка пути
    for (int k = path.size() - 1; k >= 0; k--){
        float nx = path[k].x;
        float ny = path[k].y;

        point.pose.position.x = nx - mapSize/2*mapResolution;
        point.pose.position.y = ny - mapSize/2*mapResolution;
        cout << point.pose.position.x << " " << point.pose.position.y << " " << path[k].z<< endl;

        pathMessage.poses.push_back(point);
    }

    // Отрисовка препятствий в виде кругов
    drawCircleObstacles(ROBOT_WIDTH_HALF);

    // Создание сообщения с картой
    for(int i = 0; i < mapSize*mapSize; i++){
        mapMessage.data[i] = gMap[i];
    }

    for(;;){
        // Публикация сообщений
        map_pub.publish(mapMessage);
        path_pub.publish(pathMessage);

        ros::spinOnce();
    }


}
void mapMessageInitParams(){
    mapMessage.info.height = mapSize;
    mapMessage.info.width = mapSize;
    mapMessage.info.resolution = mapResolution;
    mapMessage.info.origin.position.x = -mapSize * mapResolution/2;
    mapMessage.info.origin.position.y = -mapSize * mapResolution/2;
    mapMessage.header.frame_id = "/map";
    mapMessage.header.stamp = ros::Time::now();
    mapMessage.data.resize(mapMessage.info.height * mapMessage.info.width);
}
void pathMessageInitParams(){
    pathMessage.header.frame_id = "/map";
    pathMessage.header.stamp = ros::Time::now();
}

void drawCircleObstacles(float radius){

    // Проходим по всем координатам препятствий
    for(int i = 0; i < obstacleList.size(); i++){
        geometry_msgs::Point p0 = obstacleList.at(i);

        int x0 = p0.x/mapResolution + mapSize/2;
        int y0 = p0.y/mapResolution + mapSize/2;
        // Рисуем препятствие
        gMap[mapSize * (y0) + (x0)] = 100;


        // Отрисовка кругов
        for(int p = x0 - int(radius / mapResolution) - 1; p < x0 + int(radius/ mapResolution + 1); p++) {
            for(int q = y0 - int(radius / mapResolution - 1); q < y0 + int(radius / mapResolution + 1); q++) {
                float dist =  sqrt(pow((p - x0), 2) + pow((q - y0), 2))*mapResolution;
                if(dist > radius)
                    continue;

                // Рисуем то, что будет кругом впоследствии
                gMap[mapSize * q + p] = 100;
            }
        }
    }
}
