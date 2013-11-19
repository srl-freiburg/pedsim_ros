// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "scenarioview.h"
// → SGDiCoP
#include "sgdicop.h"
#include "logging.h"
#include "mainwindow.h"
#include "scenarioelement.h"
// → Qt
#include <QtOpenGL>


ScenarioView::ScenarioView(MainWindow* parentIn)
	: QGraphicsView(parentIn), parent(parentIn) {
	// load configuration
	QSettings settings;
	zoomFactor = settings.value("View/ZoomFactor", 1.2).toDouble();
	
	// configure graphics view
	setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);	

	// start mode: moving
	setDragMode(QGraphicsView::ScrollHandDrag);
	setRubberBandSelectionMode(Qt::ContainsItemShape);

	// set scaling
	// → flip y-axis (up=positive direction)
	scale(1, -1);
	
	// connect to signals
	connect(parent, SIGNAL(modeChanged(bool)),
		this, SLOT(modeChanged(bool)));
	connect(&SGDICOP, SIGNAL(scenarioLoaded()),
		this, SLOT(zoomToWholeScenario()));
}

ScenarioView::~ScenarioView() {
	// clean up
}

void ScenarioView::modeChanged(bool editModeIn) {
	editMode = editModeIn;
	
	if(editMode) {
		// edit mode
		// → update mode: selection
		setDragMode(QGraphicsView::RubberBandDrag);
	}
	else {
		// normal mode
		// → update mode: moving
		setDragMode(QGraphicsView::ScrollHandDrag);
	}
}

void ScenarioView::zoomin() {
	scale(zoomFactor, zoomFactor);
}

void ScenarioView::zoomout() {
	scale(1 / zoomFactor, 1 / zoomFactor);
}

void ScenarioView::zoomToWholeScenario() {
	fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
}

QPointF ScenarioView::centerOfView() const {
	return mapToScene(viewport()->rect().center());
}

void ScenarioView::setScene(QGraphicsScene* scene) {
	// connect to signals
	connect(scene, SIGNAL(selectionChanged()),
		this, SLOT(onSelectionChanged()));

	// do the usual setScene() stuff
	QGraphicsView::setScene(scene);
}

void ScenarioView::keyPressEvent(QKeyEvent* event) {
	// select the corresponding action
	if(event->matches(QKeySequence::ZoomIn)) {
		// zooming in
		zoomin();
	}
	else if(event->matches(QKeySequence::ZoomOut)) {
		// zooming out
		zoomout();
	}
	else {
		// default:
		// forward to base class
//        scene(event);
//       emit scene()->setFocus();// keyPressEvent(event);
        QGraphicsView::keyPressEvent(event);
	}
}

void ScenarioView::wheelEvent(QWheelEvent* event) {
	// get the position of the mouse before scaling, in scene coords
	QPointF pointBeforeScale(mapToScene(event->pos()));

	// get the original screen centerpoint
	QPointF screenCenter = centerOfView();

	// scale the view i.e. do the zoom
	if(event->delta() > 0) {
		// zooming in
		zoomin();
	}
	else {
		// zooming out
		zoomout();
	}

	// get the position after scaling, in scene coords
	QPointF pointAfterScale(mapToScene(event->pos()));

	// get the offset of how the screen moved
	QPointF offset = pointBeforeScale - pointAfterScale;

	// adjust to the new center for correct zooming
	QPointF newCenter = screenCenter + offset;
	centerOn(newCenter);
}

void ScenarioView::onSelectionChanged() {
	// when in edit mode, select elements
	// otherwise skip
	if(editMode == false)
		return;
	
	// iterate through the selected items and process them
	QList<QGraphicsItem*> selectedItems = scene()->selectedItems();
	QSet<ScenarioElement*> selection;
	foreach(QGraphicsItem* item, selectedItems) {
		// try to upcast
		//TODO: this doesn't work, because ScenarioElement isn't a subclass of QGraphicsItem
		ScenarioElement* element = dynamic_cast<ScenarioElement*>(item);
		if(element != NULL)
			selection.insert(element);
	}
	
	// tell the main window to change the selection
	parent->setSelection(selection);
}
