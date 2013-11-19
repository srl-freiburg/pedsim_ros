// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _loggingwidget_h_
#define _loggingwidget_h_

// Includes
#include <simulator/ui_loggingwidget.h>
// -> SGDiCoP
#include "logging.h"
// -> Qt
#include <QBasicTimer>
#include <QQueue>
#include <QSettings>
#include <QDockWidget>


class LoggingWidget : public QDockWidget,
		protected Ui::LoggingWidget {
	Q_OBJECT

	// Constructor and destructor
public:
	LoggingWidget(QWidget* parent = NULL);
	virtual ~LoggingWidget();


	// Slots
protected slots:
	void onNewLogMessage(LogMessage messageIn);

	// -> GUI elements
	void on_buttonClear_clicked();
	void on_checkScroll_stateChanged(int stateIn);


	// Methods
public:
	void timerEvent(QTimerEvent* event);

protected:
	void loadIcons();
	void updateLogMessages();


	// Attributes
protected:
	QSettings settings;

	QQueue<LogMessage*> logMessagesToAdd;
	QBasicTimer updateTimer;
	int updateInterval;
	int skipInterval;
	bool scrollToLatestElement;

	// -> icons
	QIcon infoTypeIcon;
	QIcon warningTypeIcon;
	QIcon errorTypeIcon;
	QIcon fatalTypeIcon;
	QIcon debugTypeIcon;
	QIcon unknownTypeIcon;
};

#endif
