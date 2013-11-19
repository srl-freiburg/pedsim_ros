// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#include "cell.h"

// Includes
// → Qt
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QPen>
#include <QSettings>


Cell::Cell(double x, double y, double w, double h, QGraphicsScene* gs) {
    graphicsscene = gs;

    // graphical representation
    QSettings settings;
    QColor color = settings.value("Colors/Cell", QColor(55,55,55)).value<QColor>();
    QPen pen(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    pen.setCosmetic(true);
    rect = graphicsscene->addRect(x, y, w, h, pen);
    rect->setVisible(true);
    rect->setZValue(-200);

    values.reserve(2);
//    double v = 1;
//    if ((int)(x/10) % 2 == 0)
//        v = 3;
    values.push_back(-1); // agent id (negative = no agent)
    values.push_back(0); // cell occupancy (0 = free, 1 == has obstacle0

    hasAgent = false;
    occupied = false;

    x_ = x;
    y_ = y;
}

Cell::~Cell() {
    // clean up
    // → remove graphical representation
    graphicsscene->removeItem(rect);
    delete rect;
}
