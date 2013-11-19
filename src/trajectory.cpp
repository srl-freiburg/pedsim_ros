#include "trajectory.h"
#include <iostream>
#include <cmath>

Trajectory::Trajectory(QObject *parent) :
    QObject(parent)
{
    path_.clear();
    drive_path_.clear();
}


void Trajectory::addPointEnd(Tpoint p)
{
    path_.push_back(p);
    drive_path_.push_back(p);
}

void Trajectory::addPointBegin(Tpoint p)
{
    std::cout << "Not implemented yet" << std::endl;
    //path_.push_front(p);
}

Tpoint Trajectory::getNextPoint(Tpoint p)
{
    std::vector<Tpoint>::iterator it = drive_path_.begin();
    Tpoint n = path_[0];

    int erase_point = 0;
    for ( ; it != drive_path_.end(); it++)
    {
        Tpoint q = (*it);
        double dn = sqrt((p.x-n.x)*(p.x-n.x) +  (p.y-n.y)*(p.y-n.y));
        double dq = sqrt((p.x-q.x)*(p.x-q.x) +  (p.y-q.y)*(p.y-q.y));

        if (dq < dn)
        {
            n = q;
            erase_point++;
        }
    }

    std::cout << "Erasing: " << erase_point << std::endl;

    //remove it from the store
    drive_path_.erase(drive_path_.begin() + erase_point);

    return n;
}



