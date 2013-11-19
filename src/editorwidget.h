// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _editorwidget_h_
#define _editorwidget_h_

// Includes
#include <simulator/ui_editorwidget.h>
// -> Qt
#include <QDockWidget>


// Forward Declarations
class MainWindow;


class EditorWidget : public QDockWidget,
		protected Ui::EditorWidget {
	Q_OBJECT

	// Constructor and destructor
public:
	EditorWidget(MainWindow* parentIn);
	virtual ~EditorWidget();


	// Signals
signals:
	void addAgent();
	void addObstacle();
	void addWaypoint();
	void editSelected();
	void deleteSelected();


	// Slots
public slots:
	void visibilityChanged(bool visible);
	// -> GUI elements
	void on_buttonAddAgent_clicked();
	void on_buttonAddObstacle_clicked();
	void on_buttonAddWaypoint_clicked();
	void on_buttonEdit_clicked();
	void on_buttonDelete_clicked();


	// Methods
public:
	


	// Attributes
protected:
	MainWindow* parent;
	QWidget* currentEditWidget;
};

#endif
