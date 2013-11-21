// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
//      pedsim - A microscopic pedestrian simulation system.
//      Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include "grid.h"
// -> SGDiCoP
#include "cell.h"
// → Qt
#include <QtCore/qmath.h>
#include <QPen>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QSettings>

#include <iostream>



Grid::Grid(double x, double y, double w, double h) {
    minx = x;
    miny = y;
    width = w;
    height = h;

    cells.resize(qCeil(w/CONFIG.width));
    int i = 0;
    for (double xx = x; xx < (x+w); xx += CONFIG.width) {
        QVector<Cell*>& row = cells[i];
        for (double yy = y; yy < (y+h); yy += CONFIG.height) {
            // Add an element (cell) to the row
            row.push_back(new Cell(xx, yy, CONFIG.width, CONFIG.height));
        }
        ++i;
    }

    // std::cerr << "created the grid" << std::endl;
}

Grid::~Grid() {
    // clean up
    // → remove graphical representation
    delete rect;

    // → remove cells
    BOOST_FOREACH(QVector<Cell*>& row, cells)
    {
        BOOST_FOREACH(Cell* cell, row)
        {
            delete cell;
        }
    }
}


/// Gets a value out of a cell in the grid, based on the coordinates
/// \author  chgloor
/// \date    2012-02-18
/// \return  One of the values stored in the specified cell
/// \param   x/y The coordinates
/// \param   value The requested value
QVariant Grid::getValue(double x, double y, int value) {
    if ((x-minx) < 0) return 0;
    if ((y-miny) < 0) return 0;

    unsigned int cellx = (x-minx)/CONFIG.width;
    unsigned int celly = (y-miny)/CONFIG.height;

    if (cellx >= (unsigned int)cells.size()) return 0;
    if (celly >= (unsigned int)cells[0].size()) return 0;

    return cells[cellx][celly]->values[value];
}


double Grid::getValueRaw(int x, int y, int value)
{

    return (cells[x][y]->values[value]).toDouble();
}


void Grid::setOccupied(double x, double y)
{
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = floor((x-minx)/CONFIG.width);
    unsigned int celly = ceil((y-miny)/CONFIG.height);

    if (cellx >= (unsigned int)cells.size()) return;
    if (celly >= (unsigned int)cells[0].size()) return;

    cells[cellx][celly]->occupied = true;
}

void Grid::unSetOccupied(double x, double y)
{
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = (x-minx)/CONFIG.width;
    unsigned int celly = (y-miny)/CONFIG.height;

    if (cellx >= (unsigned int)cells.size()) return;
    if (celly >= (unsigned int)cells[0].size()) return;

    cells[cellx][celly]->occupied = false;
}

bool Grid::isOccupied(double x, double y)
{
    if ((x-minx) < 0) return true;
    if ((y-miny) < 0) return true;

    unsigned int cellx = floor((x-minx)/CONFIG.width);
    unsigned int celly = ceil((y-miny)/CONFIG.height);

    if (cellx >= (unsigned int)cells.size()) return true;
    if (celly >= (unsigned int)cells[0].size()) return true;

    return cells[cellx][celly]->occupied;
}


/// Set the value of a cell, whatever that value refers to
void Grid::setValue(double x, double y, int index, int value) {
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = (x-minx)/CONFIG.width;
    unsigned int celly = (y-miny)/CONFIG.height;

    if (cellx >= (unsigned int)cells.size()) return;
    if (celly >= (unsigned int)cells[0].size()) return;

    cells[cellx][celly]->values[index] = value;
}


void Grid::clearObstacles()
{
    BOOST_FOREACH(QVector<Cell*>& row, cells)
    {
        BOOST_FOREACH(Cell* cell, row)
        {
            cell->occupied = false;
        }
    }
}