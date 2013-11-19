// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _scenarioview_h_
#define _scenarioview_h_

// Includes
// -> Qt
#include <QGraphicsView>


// Forward Declarations
class MainWindow;


class ScenarioView : public QGraphicsView {
	Q_OBJECT

	// Constructor and destructor
public:
	ScenarioView(MainWindow* parentIn);
	virtual ~ScenarioView();


	// Slots
public slots:
	void modeChanged(bool editModeIn);
	void zoomin();
	void zoomout();
	void zoomToWholeScenario();
	// → QGraphicsScene's signals
protected slots:
	void onSelectionChanged();


	// Methods
public:
	QPointF centerOfView() const;
	// → QGraphicsView overrides
	virtual void setScene(QGraphicsScene* scene);
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void wheelEvent(QWheelEvent* event);


	// Attributes
protected:
	MainWindow* parent;
	double zoomFactor;
	bool editMode;
};

#endif
