// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _grid_h_
#define _grid_h_

// Includes
// → SGDiCoP
#include "cell.h"
#include "config.h"
// → Qt
#include <QVector>
#include <QPen>
#include <QBrush>


// Forward Declarations
class QGraphicsScene;
class QGraphicsRectItem;


class Grid {
    // Constructor and Destructor
public:
    Grid(double x, double y, double w, double h, QGraphicsScene* l);
    virtual ~Grid();

    QVariant getValue(double x, double y, int value);
    double getValueRaw(int x, int y, int value);
    void setValue(double x, double y, int index, int value);

    // cell occupancy
    bool isOccupied(double x, double y);
    void setOccupied(double x, double y);
    void unSetOccupied(double x, double y);
    void clearObstacles();

    inline QVector<QVector<Cell*> > getCells() { return cells; }
    inline unsigned int sizeY() { return cells[0].size(); }
    inline unsigned int sizeX() { return cells.size(); }

    // inline 

public:

    QGraphicsScene* graphicsscene;
    QGraphicsRectItem* rect;
    double width;
    double height;
    double minx;
    double miny;

private:
    QVector<QVector<Cell*> > cells;

};

#endif
