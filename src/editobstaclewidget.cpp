// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "editobstaclewidget.h"
// → SGDiCoP
#include "mainwindow.h"
#include "obstacle.h"


EditObstacleWidget::EditObstacleWidget(MainWindow* parentIn, Obstacle* obstacleIn)
	: QDockWidget(parentIn), parent(parentIn), obstacle(obstacleIn) {
	// initialize UI elements
	setupUi(this);
	
	// initialize values
	spinboxX1->setValue(obstacle->getax());
	spinboxY1->setValue(obstacle->getay());
	spinboxX2->setValue(obstacle->getbx());
	spinboxY2->setValue(obstacle->getby());
}

EditObstacleWidget::~EditObstacleWidget() {
	// clean up
}

void EditObstacleWidget::on_spinboxX1_valueChanged(double xIn) {
	// update obstacle property
	obstacle->setX1(xIn);
}

void EditObstacleWidget::on_spinboxY1_valueChanged(double yIn) {
	// update obstacle property
	obstacle->setY1(yIn);
}

void EditObstacleWidget::on_spinboxX2_valueChanged(double xIn) {
	// update obstacle property
	obstacle->setX2(xIn);
}

void EditObstacleWidget::on_spinboxY2_valueChanged(double yIn) {
	// update obstacle property
	obstacle->setY2(yIn);
}
