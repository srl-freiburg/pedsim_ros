// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "editwaypointwidget.h"
// → SGDiCoP
#include "mainwindow.h"
#include "waypoint.h"


EditWaypointWidget::EditWaypointWidget(MainWindow* parentIn, Waypoint* waypointIn)
	: QDockWidget(parentIn), parent(parentIn), waypoint(waypointIn) {
	// initialize UI elements
	setupUi(this);
	
	// initialize values
	spinboxX->setValue(waypoint->getx());
	spinboxY->setValue(waypoint->gety());
	spinboxRadius->setValue(waypoint->getr());
}

EditWaypointWidget::~EditWaypointWidget() {
	// clean up
}

void EditWaypointWidget::on_spinboxX_valueChanged(double xIn) {
	// update waypoint property
	waypoint->setx(xIn);
}

void EditWaypointWidget::on_spinboxY_valueChanged(double yIn) {
	// update waypoint property
	waypoint->sety(yIn);
}

void EditWaypointWidget::on_spinboxRadius_valueChanged(double rIn) {
	// update waypoint property
	waypoint->setr(rIn);
}
