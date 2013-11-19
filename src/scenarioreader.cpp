// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include "scenarioreader.h"
// → SGDiCoP
#include "agent.h"
#include "obstacle.h"
#include "waypoint.h"
#include "logging.h"
// → Qt
#include <QFile>


ScenarioReader::ScenarioReader(const QSharedPointer<Scene>& sceneIn) {
    scene = sceneIn;
}


bool ScenarioReader::readFromFile(const QString& filename) {
    INFO_LOG("Loading scenario file '%1'.", filename);

    // open file
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ERROR_LOG("Couldn't open scenario file!");
        return false;
    }

    // read input
    while(!file.atEnd()) {
        QByteArray line = file.readLine();
        processData(line);
    }

    // report success
    return true;
}


/// Called for each line in the file
/// \date    2012-02-03
void ScenarioReader::processData(QByteArray& data) {
    xmlReader.addData(data);
    CONFIG.obstacle_positions.clear();

    while(!xmlReader.atEnd()) {
        xmlReader.readNext();
        if(xmlReader.isStartElement()) {
            if((xmlReader.name() == "scenario")
                    || (xmlReader.name() == "welcome")) {
                // nothing to do
            }
            else if(xmlReader.name() == "obstacle") {
                double x1 = xmlReader.attributes().value("x1").toString().toDouble();
                double y1 = xmlReader.attributes().value("y1").toString().toDouble();
                double x2 = xmlReader.attributes().value("x2").toString().toDouble();
                double y2 = xmlReader.attributes().value("y2").toString().toDouble();
                Obstacle* obs = new Obstacle(scene, x1, y1, x2, y2);
                scene->addObstacle(obs);
                drawObstacles(x1, y1, x2, y2);
            }
            else if(xmlReader.name() == "waypoint") {
                // TODO - add an explicit waypoint type
                QString id = xmlReader.attributes().value("id").toString();
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                double r = xmlReader.attributes().value("r").toString().toDouble();

                Waypoint* w = new Waypoint(scene, id, x, y, r);

                if (boost::starts_with(id, "start")) {
                    w->setType(Ped::Twaypoint::TYPE_BIRTH);
                    std::cout << "adding a birth waypoint" << std::endl;
                }

                if (boost::starts_with(id, "stop")) {
                    w->setType(Ped::Twaypoint::TYPE_DEATH);
                    std::cout << "adding a death waypoint" << std::endl;
                }


                scene->waypoints[id] = w;
            }
            else if(xmlReader.name() == "agent") {
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                int n = xmlReader.attributes().value("n").toString().toDouble();
                double dx = xmlReader.attributes().value("dx").toString().toDouble();
                double dy = xmlReader.attributes().value("dy").toString().toDouble();
                double type = xmlReader.attributes().value("type").toString().toInt();
                //TODO: keep agent group and expand later!?
                for (int i=0; i<n; i++) {
                    Agent* a = new Agent(scene);
                    double randomizedX = x;
                    double randomizedY = y;
                    // handle dx=0 or dy=0 cases
                    //TODO: qrand() actually needs to be seeded to create different runs
                    if(dx != 0)
                        randomizedX += qrand()/(double)RAND_MAX * dx - dx/2;
                    if(dy != 0)
                        randomizedY += qrand()/(double)RAND_MAX * dy - dy/2;
                    a->setPosition(randomizedX, randomizedY);
                    a->setType(type);
                    scene->addAgent(a);
                    currentAgents.append(a);
                }
            }
            else if(xmlReader.name() == "addwaypoint") {
                QString id = xmlReader.attributes().value("id").toString();
                // add waypoints to current agents, not just those of the current <agent> element
                foreach(Agent* a, currentAgents)
                    a->addWaypoint(scene->waypoints[id]);
            }
            else {
                // inform the user about invalid elements
                ERROR_LOG("Unknown element: <%1>", xmlReader.name().toString());
            }
        }
        else if(xmlReader.isEndElement()) {
            if (xmlReader.name() == "agent") {
                currentAgents.clear();
            }
        }
    }

}


// void ScenarioReader::drawObstacles(float x1, float y1, float x2, float y2){

//     int i;               // loop counter
//     int ystep, xstep;    // the step on y and x axis
//     int error;           // the error accumulated during the increment
//     int errorprev;       // *vision the previous value of the error variable
//     int y = y1, x = x1;  // the line points
//     int ddy, ddx;        // compulsory variables: the double values of dy and dx
//     int dx = x2 - x1;
//     int dy = y2 - y1;
//     double unit_x,unit_y;
//     unit_x=1;
//     unit_y=1;
//     // POINT (y1, x1);  // first point
//     // NB the last point can't be here, because of its previous point (which has to be verified)
//     if (dy < 0){
//         ystep = -unit_y;
//         dy = -dy;
//     }else
//         ystep = unit_y;
//     if (dx < 0){
//         xstep = -unit_x;
//         dx = -dx;
//     }else
//         xstep = unit_x;
//     ddy = 2 * dy;  // work with double values for full precision
//     ddx = 2 * dx;
//     QPen p(QPen(Qt::red, 0.1));
//     QBrush b(QBrush(Qt::red));
//     CONFIG.obstacle_positions.push_back(TLoc(x1,y1));
//     // Scene::grid_->changeCellColor(x1,y1,p,b);

//     if (ddx >= ddy){  // first octant (0 <= slope <= 1)
//         // compulsory initialization (even for errorprev, needed when dx==dy)
//         errorprev = error = dx;  // start in the middle of the square
//         for (i=0 ; i < dx ; i++){  // do not use the first point (already done)
//             x += xstep;
//             error += ddy;
//             if (error > ddx){  // increment y if AFTER the middle ( > )
//                 y += ystep;
//                 error -= ddx;
//                 // three cases (octant == right->right-top for directions below):
//                 if (error + errorprev < ddx){  // bottom square also
//                     CONFIG.obstacle_positions.push_back(TLoc(x,y-ystep));
//                     // Scene::grid_->changeCellColor(x,y-ystep,p,b);
//                 }
//                 else if (error + errorprev > ddx){
//                     // left square also
//                     CONFIG.obstacle_positions.push_back(TLoc(x-xstep,y));
//                     // Scene::grid_->changeCellColor(x-xstep,y,p,b);
//                 }
//                 else{  // corner: bottom and left squares also
//                     CONFIG.obstacle_positions.push_back(TLoc(x,y-ystep));
//                     // Scene::grid_->changeCellColor(x,y-ystep,p,b);

//                     CONFIG.obstacle_positions.push_back(TLoc(x-xstep,y));
//                     // Scene::grid_->changeCellColor(x-xstep,y,p,b);

//                 }
//             }
//             CONFIG.obstacle_positions.push_back(TLoc(x,y));
//             // Scene::grid_->changeCellColor(x,y,p,b);
//             errorprev = error;
//         }
//     }else{  // the same as above
//         errorprev = error = dy;
//         for (i=0 ; i < dy ; i++){
//             y += ystep;
//             error += ddx;
//             if (error > ddy){
//                 x += xstep;
//                 error -= ddy;
//                 if (error + errorprev < ddy){
//                     CONFIG.obstacle_positions.push_back(TLoc(x-xstep,y));
//                     // Scene::grid_->changeCellColor(x-xstep,y,p,b);



//                 }
//                 else if (error + errorprev > ddy){
//                     CONFIG.obstacle_positions.push_back(TLoc(x,y-ystep));
//                     // Scene::grid_->changeCellColor(x,y-ystep,p,b);


//                 }
//                 else{
//                     CONFIG.obstacle_positions.push_back(TLoc(x-xstep,y));
//                     // Scene::grid_->changeCellColor(x-xstep,y,p,b);

//                     CONFIG.obstacle_positions.push_back(TLoc(x,y-ystep));
//                     // Scene::grid_->changeCellColor(x,y-ystep,p,b);


//                 }
//             }

//             CONFIG.obstacle_positions.push_back(TLoc(x,y));
//             // Scene::grid_->changeCellColor(x,y,p,b);
//             errorprev = error;
//         }
//     }

// }

// /// draw obstacle cells using Bresenham algorithm
// /// \brief color the cell with obstacles differently
void ScenarioReader::drawObstacles(float x1, float y1, float x2, float y2)
{
    // Bresenham's line algorithm
    // TODO - update to the new Luigi modification
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 8.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            CONFIG.obstacle_positions.push_back(TLoc(y,x));
            scene->getGrid()->setOccupied(y, x);

            CONFIG.obstacle_positions.push_back(TLoc(y+CONFIG.height,x+CONFIG.width));
            scene->getGrid()->setOccupied(y+CONFIG.height, x+CONFIG.width);

            CONFIG.obstacle_positions.push_back(TLoc(y-CONFIG.height,x-CONFIG.width));
            scene->getGrid()->setOccupied(y-CONFIG.height, x-CONFIG.width);
        }
        else
        {
            CONFIG.obstacle_positions.push_back(TLoc(x,y));
            scene->getGrid()->setOccupied(x, y);

            CONFIG.obstacle_positions.push_back(TLoc(x+CONFIG.width,y+CONFIG.height));
            scene->getGrid()->setOccupied(x+CONFIG.width, y+CONFIG.height);

            CONFIG.obstacle_positions.push_back(TLoc(x-CONFIG.width,y-CONFIG.height));
            scene->getGrid()->setOccupied(x-CONFIG.width, y-CONFIG.height);
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}
