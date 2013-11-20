// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

// Includes
#include "controlwidget.h"
// → SGDiCoP
#include "config.h"
#include "sgdicop.h"


ControlWidget::ControlWidget(QWidget* parent)
	: QDockWidget(parent) {
	// initialize UI elements
	setupUi(this);

	connect(waypointsCheckBox, SIGNAL(toggled(bool)), this, SLOT(showWaypoints(bool)));

	connect(showForcesCheckBox, SIGNAL(toggled(bool)), this, SLOT(showForces(bool)));
	connect(showDirectionCheckBox, SIGNAL(toggled(bool)), this, SLOT(showDirection(bool)));
    connect(checkboxTeleop, SIGNAL(toggled(bool)), this, SLOT(enableTeleop(bool)));

	connect(mlLookAheadCheckBox, SIGNAL(toggled(bool)), this, SLOT(mlLookAhead(bool)));

	connect(wallforceSlider, SIGNAL(valueChanged(int)), this, SLOT(simWallForce(int)));
	connect(pedforceSlider, SIGNAL(valueChanged(int)), this, SLOT(simPedForce(int)));
	connect(simspeedSlider, SIGNAL(valueChanged(int)), this, SLOT(simSpeed(int)));
	connect(simhSlider, SIGNAL(valueChanged(int)), this, SLOT(simh(int)));
    connect(slider_robot_speed, SIGNAL(valueChanged(int)), this, SLOT(controlRobotSpeed(int)));
    connect(slider_robot_direction, SIGNAL(valueChanged(int)), this, SLOT(controlRobotDirection(int)));

	connect(zoominButton, SIGNAL(clicked()), this, SIGNAL(zoomin()));
	connect(zoomoutButton, SIGNAL(clicked()), this, SIGNAL(zoomout()));
	
	// update elements
	updatePauseButton();
}


void ControlWidget::on_buttonPause_clicked(bool checked) {
	// either pause or resume
	if(checked) {
		// resume
		SGDICOP.unpause();
	}
	else {
		// pause
		SGDICOP.pause();
	}

	// update button
	updatePauseButton();
}


void ControlWidget::setFps(double fps) {
	fpsLabel->setText(QString("FPS (req %0 curr %1)").arg(simspeedSlider->value()).arg(fps, 0, 'f', 1));
}


void ControlWidget::showWaypoints(bool show) {
	CONFIG.setGuiShowWaypoints(show);
}


void ControlWidget::simWallForce(int value) {
	CONFIG.setSimWallForce(value/10.0);
	wallforceLabel->setText(QString("Wall Force (%1)").arg(CONFIG.simWallForce));
}


void ControlWidget::simPedForce(int value) {
	CONFIG.setSimPedForce(value/10.0);
	pedforceLabel->setText(QString("Pedestrian Force (%1)").arg(CONFIG.simPedForce));
}


void ControlWidget::simSpeed(int value) {
	CONFIG.setSimSpeed(1000.0/value);
}


void ControlWidget::mlLookAhead(bool value) {
	CONFIG.mlLookAhead = value;
}


void ControlWidget::showForces(bool show) {
	CONFIG.showForces = show;
}


void ControlWidget::showDirection(bool show) {
	CONFIG.showDirection = show;
}

void ControlWidget::enableTeleop(bool enable)
{
    std::cout << "WARN: This no longer works, move on" << std::endl; 
}


void ControlWidget::simh(int value) {
	CONFIG.simh = value/100.0;
	simhLabel->setText(QString("Precision (h=%1)").arg(CONFIG.simh));
}

void ControlWidget::updatePauseButton() {
	if(SGDICOP.isPaused()) {
		buttonPause->setText(tr("Resume"));
		buttonPause->setChecked(false);
		QIcon startIcon = QIcon::fromTheme("media-playback-start");
		if(!startIcon.isNull())
			buttonPause->setIcon(startIcon);
	}
	else {
		buttonPause->setText(tr("Pause"));
		buttonPause->setChecked(true);
		QIcon startIcon = QIcon::fromTheme("media-playback-pause");
		if(!startIcon.isNull())
			buttonPause->setIcon(startIcon);
	}
}


void ControlWidget::controlRobotSpeed(int value)
{
    std::cout << "WARN: This no longer works, move on" << std::endl; 
}

void ControlWidget::controlRobotDirection(int value)
{
   std::cout << "WARN: This no longer works, move on" << std::endl; 
}
