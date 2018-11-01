#ifndef RRT_H
#define RRT_H

#include <geometry_msgs/Point.h>

#include <math.h>
#include <vector>
#include <iostream>

#include <random>

#include "node.h"
#include "dubinspathplanning.h"

using namespace std;

class RRT
{
private:
    // Вектор с координатами препятствий
    vector<geometry_msgs::Point> obstacleList;

    // Размер и разршение карты
    float mapSize;
    float mapResolution;
    // Начало карты(0) и конец карты = mapSize/mapResolution
    int minRand;
    int maxRand;

    // Координаты старта и финиша
    Node* start;
    Node* end;

    int goalSampleRate;
    // Количество итераций
    int maxIter;

    // Минимально возможное расстояния до препятствия
    float minDistToObstacle;

    // Лист с открытыми узлами и путями к ним
    vector<Node*> nodeList;
    // Радиус поворота робота
    float curvature;

    // Сгенерировать рандомный узел
    Node* getRandomPoint();
    // Получить ближайший индекс узла
    int getNearestListIndex(Node* rnd);
    // Проложить путь до рандомного узла
    Node* steer(Node* rnd, int nind);
    // Проверка на пересечение с препятствиями
    bool collisionCheck(Node* node);
    // Выбрать родителя
    Node* choose_parent(Node* newNode, vector<int> nearInds);
    // Переписать узел, если это возможно
    void rewire(vector<int> nearinds);
    // Найти ближайшие узлы
    vector<int> find_near_nodes(Node* newNode);

    // Сфлормировать окончательный курс
    vector<geometry_msgs::Point> gen_final_course(int goalInd);
    // Получить лучший индекс
    float get_best_last_index();

    // Рассчитать расстояние до цели
    float calc_dist_to_goal(float x, float y);
    // Перевести метры в ячейки для планировщика
    float metrs2cells(float metrs);
    // Отбросить лишний угол и войти в диапазон [-PI;PI]
    float pi_2_pi(float angle);

public:
    RRT();

    // Начать планирование
    vector<geometry_msgs::Point> Planning(geometry_msgs::Point s, geometry_msgs::Point g,
                                          vector<geometry_msgs::Point> ob, float curv,
                                          float robot_width_half,float mapSize0,
                                          float mapResolution = 0.04, int maxIter0 = 100);
};

#endif // RRT_H
