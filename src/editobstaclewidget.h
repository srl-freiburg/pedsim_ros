// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _editobstaclewidget_h_
#define _editobstaclewidget_h_

// Includes
#include <simulator/ui_editobstaclewidget.h>
// -> Qt
#include <QDockWidget>


// Forward Declarations
class MainWindow;
class Obstacle;


class EditObstacleWidget : public QDockWidget,
		protected Ui::EditObstacleWidget {
	Q_OBJECT

	// Constructor and destructor
public:
	EditObstacleWidget(MainWindow* parentIn, Obstacle* obstacleIn);
	virtual ~EditObstacleWidget();


	// Slots
protected slots:
	void on_spinboxX1_valueChanged(double xIn);
	void on_spinboxY1_valueChanged(double yIn);
	void on_spinboxX2_valueChanged(double xIn);
	void on_spinboxY2_valueChanged(double yIn);


	// Methods
public:
	


	// Attributes
protected:
	MainWindow* parent;
	Obstacle* obstacle;
};

#endif
