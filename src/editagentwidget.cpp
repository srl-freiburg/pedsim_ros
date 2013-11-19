// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "editagentwidget.h"
// → SGDiCoP
#include "mainwindow.h"
#include "agent.h"
#include "waypoint.h"


EditAgentWidget::EditAgentWidget(MainWindow* parentIn, Agent* agentIn)
	: QDockWidget(parentIn), parent(parentIn), agent(agentIn) {
	// initialize UI elements
	setupUi(this);
	
	// initialize values
	spinboxX->setValue(agent->getx());
	spinboxY->setValue(agent->gety());
	//TODO: there is no way to get VMax from Ped::Tagent!
// 	spinboxVMax->setValue(agent->getVMax());
	spinboxType->setValue(agent->gettype());
	
	// → add waypoints
	foreach(Waypoint* waypoint, agent->waypoints)
		listWaypoints->addItem(waypoint->toString());
}

EditAgentWidget::~EditAgentWidget() {
	// clean up
}

void EditAgentWidget::on_spinboxX_valueChanged(double xIn) {
	// update agent property
	agent->setX(xIn);
}

void EditAgentWidget::on_spinboxY_valueChanged(double yIn) {
	// update agent property
	agent->setY(yIn);
}

void EditAgentWidget::on_spinboxVMax_valueChanged(double vmaxIn) {
	// update agent property
	agent->setVmax(vmaxIn);
}

void EditAgentWidget::on_spinboxType_valueChanged(int typeIn) {
	// update agent property
	agent->setType(typeIn);
}
