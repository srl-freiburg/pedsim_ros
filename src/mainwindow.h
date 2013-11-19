// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _mainwindow_h_
#define _mainwindow_h_

// Includes
// → Qt
#include <QMainWindow>
#include <QSet>
#include <QSettings>

// Forward Declarations
class ControlWidget;
class EditorWidget;
class LoggingWidget;
class ScenarioElement;
class ScenarioView;
class QAction;
class QMenu;
class QGraphicsScene;


class MainWindow : public QMainWindow {
	Q_OBJECT

	// Constructor and Destructor
public:
	MainWindow();


	// Signals
signals:
	void modeChanged(bool editMode);
	void selectionChanged();
	//TODO: signals about changed scene (new, loaded, updated)


	// Slots
public slots:
	bool newScenario();
	bool openScenario();
	bool saveScenario();
	void enterEditMode();
	void leaveEditMode();
	// → SGDiCoP Signals
	void onScenarioCreated();
	void onScenarioLoaded();

protected slots:
	void about();


	// Methods
public:
	QPointF getDefaultPosition() const;
	// → Input file
	QString showOpenScenarioDialog();
	// → Selection
	void clearSelection(bool informUserIn = true);
	void setSelection(ScenarioElement* elementIn);
	void setSelection(const QSet<ScenarioElement*>& elementsIn);
	const QSet<ScenarioElement*>& getSelection() const;
	void addToSelection(ScenarioElement* elementIn);
	bool removeFromSelection(ScenarioElement* elementIn);
protected:
	bool verifyQuitting();
	void createActions();
	void createMenus();
// 	void createToolBars();
// 	void createStatusBar();
	void loadIcons();
	void updateWindowTitle();
	void readSettings();
	void writeSettings();

	// → QMainWindow Overrides
	void closeEvent(QCloseEvent* event);
	void resizeEvent(QResizeEvent* event);


	// Attributes
public:
	ScenarioView* graphicsView;
	QGraphicsScene* graphicsscene;
protected:
	QSettings settings;
	QMenu* fileMenu;
	QMenu* viewMenu;
	QMenu* helpMenu;
// 	QToolBar* editToolBar;
	QAction* newAction;
	QAction* openAction;
	QAction* saveAction;
	QAction* exitAction;
	QAction* aboutAction;
// 	QAction* zoominAction;
// 	QAction* zoomoutAction;

	ControlWidget* uicontrol;
	// LoggingWidget* uilogging;
	EditorWidget* uieditor;
	
	QSet<ScenarioElement*> selection;
};

#endif
