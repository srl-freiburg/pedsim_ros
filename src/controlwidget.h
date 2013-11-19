// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

#ifndef _controlwidget_h_
#define _controlwidget_h_

// Includes
// → UI
#include <simulator/ui_controlwidget.h>


class ControlWidget : public QDockWidget, protected Ui::ControlWidget {
	Q_OBJECT

	// Constructor and Destructor
public:
	ControlWidget(QWidget* parent = NULL);


	// Signals
signals:
	// → Zooming
	void zoomin();
	void zoomout();


	// Slots
public slots:
	void on_buttonPause_clicked(bool checked);
	void showWaypoints(bool show);
	void simWallForce(int value);
	void simPedForce(int value);
	void simSpeed(int value);
	void simh(int value);
	void mlLookAhead(bool value);
	void showForces(bool show);
	void showDirection(bool show);
    void enableTeleop(bool enable);
	void setFps(double fps);
    void controlRobotSpeed(int value);
    void controlRobotDirection(int value);


	// Methods
	void updatePauseButton();
private slots:

};

#endif
