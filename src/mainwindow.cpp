// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include "mainwindow.h"
// → SGDiCoP
#include "sgdicop.h"
#include "controlwidget.h"
#include "loggingwidget.h"
#include "editorwidget.h"
#include "scenarioview.h"
#include "scenarioelement.h"
// → Qt
#include <QCloseEvent>
#include <QFileDialog>
#include <QGraphicsScene>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>


MainWindow::MainWindow() {
	// create graphical representation
	graphicsscene = new QGraphicsScene();
	QColor backgroundColor = settings.value("Colors/Background", Qt::black).value<QColor>();
	graphicsscene->setBackgroundBrush(backgroundColor);
	graphicsscene->setItemIndexMethod(QGraphicsScene::NoIndex);
	
	graphicsView = new ScenarioView(this);
	graphicsView->setScene(graphicsscene);
	graphicsView->show();
	setCentralWidget(graphicsView);

	// add control widget to the GUI
	uicontrol = new ControlWidget(this);
	addDockWidget(Qt::LeftDockWidgetArea, uicontrol);

	// add logging widget to the GUI
	// uilogging = new LoggingWidget(this);
	// addDockWidget(Qt::BottomDockWidgetArea, uilogging);

	// add editor widget to the GUI
	uieditor = new EditorWidget(this);
	addDockWidget(Qt::RightDockWidgetArea, uieditor);
	uieditor->setVisible(false);

	createActions();
	loadIcons();
	createMenus();
	
	readSettings();
    setFocusPolicy(Qt::StrongFocus);

	// connect signals
	connect(uicontrol, SIGNAL(zoomin()), graphicsView, SLOT(zoomin()));
	connect(uicontrol, SIGNAL(zoomout()), graphicsView, SLOT(zoomout()));
	connect(&SGDICOP, SIGNAL(scenarioCreated()), this, SLOT(onScenarioCreated()));
	connect(&SGDICOP, SIGNAL(scenarioLoaded()), this, SLOT(onScenarioLoaded()));
}

QPointF MainWindow::getDefaultPosition() const {
	if(graphicsView == NULL)
		return QPointF();
	else
		return graphicsView->centerOfView();
}

QString MainWindow::showOpenScenarioDialog() {
	// open a file selector to choose the scene input file
	QSettings settings;
	QString recentInputFilename = settings.value("RecentInputFile").toString();
	QString sceneInputFilename = QFileDialog::getOpenFileName(
		this,
		tr("Open scenario file"),
		recentInputFilename,
		tr("SGDiCoP Scenario Files (*.xml *.scenario);;All (*)"));

	// aborted?
	if(!sceneInputFilename.isEmpty()) {
		// save selection as recent file
		settings.setValue("RecentInputFile", sceneInputFilename);
	}
	
	return sceneInputFilename;
}
	
void MainWindow::clearSelection(bool informUserIn) {
	// deselect elements
	foreach(ScenarioElement* element, selection)
		element->setSelected(false);

	selection.clear();
	
	// inform users
	if(informUserIn)
		emit selectionChanged();
}

void MainWindow::setSelection(ScenarioElement* elementIn) {
	clearSelection(false);
	selection.insert(elementIn);

	// select element
	elementIn->setSelected(true);

	// inform users
	emit selectionChanged();
}

void MainWindow::setSelection(const QSet<ScenarioElement*>& elementsIn) {
	clearSelection(false);
	selection = elementsIn;

	// select element
	foreach(ScenarioElement* element, selection)
		element->setSelected(true);

	// inform users
	emit selectionChanged();
}

const QSet<ScenarioElement*>& MainWindow::getSelection() const {
	return selection;
}

void MainWindow::addToSelection(ScenarioElement* elementIn) {
	selection.insert(elementIn);

	// select element
	elementIn->setSelected(true);

	// inform users
	emit selectionChanged();
}

bool MainWindow::removeFromSelection(ScenarioElement* elementIn) {
	bool found = selection.remove(elementIn);

	// deselect element
	elementIn->setSelected(false);

	// inform users, if selection really changed
	if(found)
		emit selectionChanged();
	
	return found;
}

void MainWindow::closeEvent(QCloseEvent* event) {
	// ask user whether he really wants to quit
	bool reallyQuit = verifyQuitting();
	if(reallyQuit == false) {
		event->ignore();
		return;
	}

	// quit
	SGDICOP.quit();
	event->accept();
}

bool MainWindow::verifyQuitting() {
	// popup if the window is minimized
	if(isMinimized())
		setWindowState(windowState() & ~Qt::WindowMinimized);

	// QMessageBox::StandardButton buttonPressed = QMessageBox::question(this,
	// 	tr("Quit?"),
	// 	tr("Do you really want to quit this application?"),
	// 	QMessageBox::Yes|QMessageBox::Cancel,
	// 	QMessageBox::Cancel);

	// return (buttonPressed == QMessageBox::Yes);
	return true;
}

void MainWindow::about() {
	QMessageBox::about(
		this,
		tr("About Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)"),
		tr("<b>Simulating Group Dynamics in Crowds of Pedestrians</b> (SGDiCoP) is a pedestrian simulation system "
			"with group dynamics in mind.<br>"
			"It is based on <b>libpedsim</b>, http://pedsim.silmaril.org/<br>"
			" Original Copyright: (c) 2003-2012 by Christian Gloor"
			" Additions: (c) 2013 by Sven Wehner (mail@svenwehner.de)"));
}

void MainWindow::createActions() {
	newAction = new QAction(tr("&New"), this);
	newAction->setShortcut(tr("Ctrl+N"));
	newAction->setStatusTip(tr("Create new scenario"));
	connect(newAction, SIGNAL(triggered()), this, SLOT(newScenario()));

	openAction = new QAction(tr("&Open"), this);
	openAction->setShortcut(tr("Ctrl+O"));
	openAction->setStatusTip(tr("Exit the application"));
	connect(openAction, SIGNAL(triggered()), this, SLOT(openScenario()));

	saveAction = new QAction(tr("&Save"), this);
	saveAction->setShortcut(tr("Ctrl+S"));
	saveAction->setStatusTip(tr("Exit the application"));
	connect(saveAction, SIGNAL(triggered()), this, SLOT(saveScenario()));

	exitAction = new QAction(tr("E&xit"), this);
	exitAction->setShortcut(tr("Ctrl+Q"));
	exitAction->setStatusTip(tr("Exit the application"));
	connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

	aboutAction = new QAction(tr("&About"), this);
	aboutAction->setStatusTip(tr("Show the application's About box"));
	connect(aboutAction, SIGNAL(triggered()), this, SLOT(about()));

// 	zoominAction = new QAction(QIcon(":/images/zoomin.png"), tr("Zoom In"), this);
// 	zoominAction->setStatusTip(tr("Zoom In"));
// 	connect(zoominAction, SIGNAL(triggered()), this, SLOT(zoomin()));
// 
// 	zoomoutAction = new QAction(QIcon(":/images/zoomout.png"), tr("Zoom Out"), this);
// 	zoomoutAction->setStatusTip(tr("Zoom Out"));
// 	connect(zoomoutAction, SIGNAL(triggered()), this, SLOT(zoomout()));
}

void MainWindow::createMenus() {
	fileMenu = menuBar()->addMenu(tr("&File"));
	fileMenu->addAction(newAction);
	fileMenu->addAction(openAction);
	fileMenu->addAction(saveAction);
	fileMenu->addSeparator();
	fileMenu->addAction(exitAction);
	
	viewMenu = menuBar()->addMenu(tr("&View"));
	viewMenu->addAction(uicontrol->toggleViewAction());
	// viewMenu->addAction(uilogging->toggleViewAction());
	viewMenu->addAction(uieditor->toggleViewAction());

	menuBar()->addSeparator();

	helpMenu = menuBar()->addMenu(tr("&Help"));
	helpMenu->addAction(aboutAction);
}

// void MainWindow::createToolBars() {
// 	//TODO: this isn't used
// // 	editToolBar = addToolBar(tr("Zoom"));
// // 	editToolBar->addAction(zoominAction);
// // 	editToolBar->addAction(zoomoutAction);
// }
// 
// 
// void MainWindow::createStatusBar() {
// 	//TODO: this isn't really /*necessary*/ → disabled
// 	statusBar()->showMessage(tr("Ready"));
// }

void MainWindow::loadIcons() {
	// File
	//  -> New
	QIcon newIcon = QIcon::fromTheme("document-new");
	if(!newIcon.isNull())
		newAction->setIcon(newIcon);
	//  -> Open
	QIcon openIcon = QIcon::fromTheme("document-open");
	if(!openIcon.isNull())
		openAction->setIcon(openIcon);
	//  -> Save Graph
	QIcon saveIcon = QIcon::fromTheme("document-save");
	if(!saveIcon.isNull())
		saveAction->setIcon(saveIcon);
	//  -> Quit
	QIcon quitIcon = QIcon::fromTheme("application-exit");
	if(!quitIcon.isNull())
		exitAction->setIcon(quitIcon);

	// Help
	//  -> About
	QIcon aboutIcon = QIcon::fromTheme("help-about");
	if(!aboutIcon.isNull())
		aboutAction->setIcon(aboutIcon);
}

void MainWindow::updateWindowTitle() {
	QString fileName = SGDICOP.scenarioFileName;
	QString title;
	if(fileName.isEmpty())
		title = tr("SGDiCoP - New Scenario");
	else
		title = tr("SGDiCoP - '%1'").arg(fileName);
	
	// apply new window title
	setWindowTitle(title);
}

void MainWindow::readSettings() {
	//TODO: default to maximize
	QPoint defaultPosition(0, 0);
	QSize defaultSize(1280, 800);

	// read settings
	QSettings settings;
	QPoint pos = settings.value("Window/Position", defaultPosition).toPoint();
	QSize size = settings.value("Window/Size", defaultSize).toSize();

	// apply settings
	resize(size);
	move(pos);
}

void MainWindow::writeSettings() {
	QSettings settings;
	settings.setValue("Window/Position", pos());
	settings.setValue("Window/Size", size());
}

void MainWindow::resizeEvent(QResizeEvent* event) {
	//write settings
	writeSettings();
}

bool MainWindow::newScenario() {
	// close previous scene
	return SGDICOP.newScene();
}

bool MainWindow::openScenario() {
	// open a file selector to choose the scene input file
	QString sceneInputFilename = showOpenScenarioDialog();
	
	// abort?
	if(sceneInputFilename.isEmpty())
		return false;

	// close previous scene
	SGDICOP.clearScene();

	// open scenario
	bool loadResult = SGDICOP.loadScene(sceneInputFilename);

	// inform the user
	if(loadResult)
		INFO_LOG("Successfully loaded scenario file '%1'", sceneInputFilename);
	else
		ERROR_LOG("Couldn't load scenario file '%1'", sceneInputFilename);

	return loadResult;
}

bool MainWindow::saveScenario() {
	QSettings settings;
	QString recentOutputFilename = settings.value("RecentOutputFile").toString();

	// if there hasn't been a previous output file name, use the current file name
	if(recentOutputFilename.isEmpty()) {
		QFileInfo fileInfo(SGDICOP.scenarioFileName);
		recentOutputFilename = fileInfo.canonicalFilePath();
	}

	// open a file selector to choose the graph output file
	QString sceneOutputFilename = QFileDialog::getSaveFileName(
		this,
		tr("Save scenario file"),
		recentOutputFilename,
		tr("Scenario Files (*.xml *.scenario);;All (*)"));

	// abort?
	if(sceneOutputFilename.isEmpty())
		return false;

	// save recent output file name
	settings.setValue("RecentOutputFile", sceneOutputFilename);
	
	// write scenario to file
	bool writeResult = SGDICOP.writeScene(sceneOutputFilename);

	return writeResult;
}

void MainWindow::enterEditMode() {
	// inform others
	// HACK: true=EditMode
	emit modeChanged(true);
}

void MainWindow::leaveEditMode() {
	// inform others
	// HACK: false=NormalMode
	emit modeChanged(false);
}

void MainWindow::onScenarioCreated() {
	updateWindowTitle();
}

void MainWindow::onScenarioLoaded() {
	updateWindowTitle();
}
