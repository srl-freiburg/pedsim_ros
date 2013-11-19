// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _editagentwidget_h_
#define _editagentwidget_h_

// Includes
#include <simulator/ui_editagentwidget.h>
// -> Qt
#include <QDockWidget>


// Forward Declarations
class MainWindow;
class Agent;


class EditAgentWidget : public QDockWidget,
		protected Ui::EditAgentWidget {
	Q_OBJECT

	// Constructor and destructor
public:
	EditAgentWidget(MainWindow* parentIn, Agent* agentIn);
	virtual ~EditAgentWidget();


	// Slots
protected slots:
	void on_spinboxX_valueChanged(double xIn);
	void on_spinboxY_valueChanged(double yIn);
	void on_spinboxVMax_valueChanged(double vmaxIn);
	void on_spinboxType_valueChanged(int typeIn);


	// Methods
public:
	

	// Attributes
protected:
	MainWindow* parent;
	Agent* agent;
};

#endif
