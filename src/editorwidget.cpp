// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "editorwidget.h"
// → SGDiCoP
#include "sgdicop.h"
#include "logging.h"
#include "mainwindow.h"
#include "scene.h"
#include "agent.h"
#include "obstacle.h"
#include "waypoint.h"
#include "editagentwidget.h"
#include "editobstaclewidget.h"
#include "editwaypointwidget.h"
// → Qt
#include <QGraphicsScene>
#include <QInputDialog>
#include <QMessageBox>


EditorWidget::EditorWidget(MainWindow* parentIn)
	: QDockWidget(parentIn), parent(parentIn), currentEditWidget(NULL) {
	// initialize UI elements
	setupUi(this);
	
	//HACK: we want to be informed when the dock becomes visible
	//      is this really the best way to do this?
	connect(this, SIGNAL(visibilityChanged(bool)), 
		this, SLOT(visibilityChanged(bool)));
}

EditorWidget::~EditorWidget() {
	// clean up
}

void EditorWidget::visibilityChanged(bool visible) {
	// when becoming visible, enable edit mode
	if(visible) {
		// enter edit mode
		if(parent != NULL)
			parent->enterEditMode();
	}
	else {
		// leave edit mode
		if(parent != NULL)
			parent->leaveEditMode();
	}
}

void EditorWidget::on_buttonAddAgent_clicked() {
	// add an agent
	Agent* agent = new Agent(SGDICOP.scene);
	//TODO: add agent at the center of the ScenarioView
	QPointF position = parent->getDefaultPosition();
	agent->setPosition(position);
	SGDICOP.scene->addAgent(agent);

	// select new agent
	parent->setSelection(agent);

	// show edit dialog
	// → remove all previous widgets
	if(currentEditWidget != NULL)
		delete currentEditWidget;
	currentEditWidget = new EditAgentWidget(parent, agent);
	layoutEditElement->addWidget(currentEditWidget);
}

void EditorWidget::on_buttonAddObstacle_clicked() {
	// add an obstacle
	Obstacle* obstacle = new Obstacle(SGDICOP.scene);
	// add obstacle at the center of the ScenarioView
	QPointF centerPosition = parent->getDefaultPosition();
	QPointF endPosition = centerPosition + QPointF(1, 1);
	obstacle->setPosition(centerPosition, endPosition);
	SGDICOP.scene->addObstacle(obstacle);

	// select new obstacle
	parent->setSelection(obstacle);

	// show edit dialog
	// → remove all previous widgets
	if(currentEditWidget != NULL)
		delete currentEditWidget;
	currentEditWidget = new EditObstacleWidget(parent, obstacle);
	layoutEditElement->addWidget(currentEditWidget);
}

void EditorWidget::on_buttonAddWaypoint_clicked() {
	static int runningIdSuffix = 1;
	
	// ask the user for an ID
	QString id = tr("waypoint%1").arg(runningIdSuffix);
	bool validID = false;
	bool ok = false;
	do {
		id = QInputDialog::getText(
			this,
			tr("Waypoint ID"),
			tr("Please give the new waypoint an unique ID:"),
			QLineEdit::Normal,
			id,
			&ok);
		
		// check whether the user canceled
		if(!ok)
			return;
		
		// check whether new id is valid
		validID = !SGDICOP.scene->waypoints.contains(id);
	} while(!validID);
	++runningIdSuffix;
	
	// add an waypoint
	Waypoint* waypoint = new Waypoint(SGDICOP.scene, id);
	// add waypoint at the center of the ScenarioView
	QPointF position = parent->getDefaultPosition();
	waypoint->setPosition(position);
//	SGDICOP.scene->addWaypoint(waypoint);

	// select new waypoint
	parent->setSelection(waypoint);

	// show edit dialog
	// → remove all previous widgets
	if(currentEditWidget != NULL)
		delete currentEditWidget;
	currentEditWidget = new EditWaypointWidget(parent, waypoint);
	layoutEditElement->addWidget(currentEditWidget);
}

void EditorWidget::on_buttonEdit_clicked() {
	// remove all previous widgets
	if(currentEditWidget != NULL)
		delete currentEditWidget;
	
	// show edit dialog
	QList<QGraphicsItem*> selection = parent->graphicsscene->selectedItems();
	if(selection.size() == 0) {
		//TODO: there's nothing to edit (inform the user)
		return;
	}
	// → use only the first element
	//TODO
	QGraphicsItem* selectedElement = selection.first();
	Agent* selectedAgent = qgraphicsitem_cast<Agent*>(selectedElement);
	Obstacle* selectedObstacle = qgraphicsitem_cast<Obstacle*>(selectedElement);
	Waypoint* selectedWaypoint = qgraphicsitem_cast<Waypoint*>(selectedElement);
	if(selectedAgent != NULL)
		currentEditWidget = new EditAgentWidget(parent, selectedAgent);
	else if(selectedObstacle != NULL)
		currentEditWidget = new EditObstacleWidget(parent, selectedObstacle);
	else if(selectedWaypoint != NULL)
		currentEditWidget = new EditWaypointWidget(parent, selectedWaypoint);
	else {
		ERROR_LOG("Cannot edit selected item of unknown type!");
		return;
	}
	layoutEditElement->addWidget(currentEditWidget);
	

// 	QSet<ScenarioElement*> selectionSet = parent->getSelection();
// 	if(selectionSet.size() == 0) {
// 		//TODO: there's nothing to edit (inform the user)
// 		return;
// 	}
// 	// → use only the first element
// 	ScenarioElement* selectedElement = *(selectionSet.constBegin());
// 	if(selectionSet.size() > 1)
// 		parent->setSelection(selectedElement);
// 	// → depends on type of selected element
// 	//TODO: add type() method to ScenarioElement?
// 	Agent* selectedAgent = dynamic_cast<Agent*>(selectedElement);
// 	Obstacle* selectedObstacle = dynamic_cast<Obstacle*>(selectedElement);
// 	Waypoint* selectedWaypoint = dynamic_cast<Waypoint*>(selectedElement);
// 	if(selectedAgent != NULL)
// 		currentEditWidget = new EditAgentWidget(parent, selectedAgent);
// 	else if(selectedObstacle != NULL)
// 		currentEditWidget = new EditObstacleWidget(parent, selectedObstacle);
// 	else if(selectedWaypoint != NULL)
// 		currentEditWidget = new EditWaypointWidget(parent, selectedWaypoint);
// 	else {
// 		ERROR_LOG("Cannot edit selected item of unknown type!");
// 		return;
// 	}
// 	layoutEditElement->addWidget(currentEditWidget);
}

void EditorWidget::on_buttonDelete_clicked() {
	// show confirmation dialog
	QMessageBox::StandardButton buttonPressed = QMessageBox::question(this,
		tr("Delete?"),
		tr("Do you really want to delete the selected element(s)?"),
		QMessageBox::Yes|QMessageBox::No,
		QMessageBox::No);
	if(buttonPressed != QMessageBox::Yes)
		return;

	// remove element from scene
	//TODO
}
