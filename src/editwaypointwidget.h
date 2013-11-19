// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _editwaypointwidget_h_
#define _editwaypointwidget_h_

// Includes
#include <simulator/ui_editwaypointwidget.h>
// -> Qt
#include <QDockWidget>


// Forward Declarations
class MainWindow;
class Waypoint;


class EditWaypointWidget : public QDockWidget,
		protected Ui::EditWaypointWidget {
	Q_OBJECT

	// Constructor and destructor
public:
	EditWaypointWidget(MainWindow* parentIn, Waypoint* waypointIn);
	virtual ~EditWaypointWidget();


	// Slots
protected slots:
	void on_spinboxX_valueChanged(double xIn);
	void on_spinboxY_valueChanged(double yIn);
	void on_spinboxRadius_valueChanged(double rIn);


	// Methods
public:
	


	// Attributes
protected:
	MainWindow* parent;
	Waypoint* waypoint;
};

#endif
