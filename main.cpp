#include <QCoreApplication>
#include "dubinspathplanning.h"

int main(int argc, char *argv[])
{

    DubinsPathPlanning p;
    DubinsPathPlanning::originPath res = p.dubins_path_planning(1, 1, 45, 10, 10, 90, 1);
    for (int i = 0; i < res.px.size(); i++){
        cout  << res.px[i]  << " " << res.py[i] << endl;
    }
            return 0;
}
