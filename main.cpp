#include <QCoreApplication>
#include "dubinspathplanning.h"

int main(int argc, char *argv[])
{

    DubinsPathPlanning p;
    DubinsPathPlanning::originPath res = p.dubins_path_planning(1, 1, 45, 10, 10, 90, 1);
    for (int i = 0; i < res.x.size(); i++){
        cout  << res.x[i]  << " " << res.y[i] << endl;
    }
            return 0;
}
