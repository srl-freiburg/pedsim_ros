// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "loggingwidget.h"


LoggingWidget::LoggingWidget(QWidget* parent)
	: QDockWidget(parent) {
	// initialize UI elements
	setupUi(this);

	// set the default sort to the time column
	tableWidget->sortByColumn(0, Qt::AscendingOrder);

	// subscribe for logging messages
	qRegisterMetaType<LogMessage>("LogMessage");
	connect(&LOGGER, SIGNAL(onNewLogMessage(LogMessage)), this, SLOT(onNewLogMessage(LogMessage)));

	// read settings
	updateInterval = settings.value("LoggingWidget/updateInterval", 500).toInt();
	skipInterval = settings.value("LoggingWidget/skipInterval", 250).toInt();
	scrollToLatestElement = settings.value("LoggingWidget/scrollToLatestElement", true).toBool();

	// set GUI elements accordingly
	checkScroll->setCheckState(scrollToLatestElement?Qt::Checked:Qt::Unchecked);
	loadIcons();

	// let the interface get updated after a period (default: 500ms)
	updateTimer.start(updateInterval, this);
}

LoggingWidget::~LoggingWidget() {
	// clean up
}

void LoggingWidget::onNewLogMessage(LogMessage messageIn) {
	// add log message to the queue
	logMessagesToAdd.enqueue(new LogMessage(messageIn));
}

void LoggingWidget::on_buttonClear_clicked() {
	// remove all messages from the table view
	// Note: we can't use clear() since it would remove the header, too
	// Note: we can't use clearContents() since it would leave the empty rows
	while(tableWidget->rowCount() > 0)
		tableWidget->removeRow(0);
}

void LoggingWidget::on_checkScroll_stateChanged(int stateIn) {
	// enables scrolling if the check box is checked
	scrollToLatestElement = (stateIn == Qt::Checked);

	// save status
	settings.setValue("LoggingWidget/scrollToLatestElement", scrollToLatestElement);
}

void LoggingWidget::timerEvent(QTimerEvent* event) {
	if(event->timerId() != updateTimer.timerId())
		return;

	// -> update log messages
	updateLogMessages();
}

void LoggingWidget::loadIcons() {
	QIcon clearIcon = QIcon::fromTheme("edit-clear");
	if(!clearIcon.isNull())
		buttonClear->setIcon(clearIcon);

	// preload icons for messages
	infoTypeIcon = QIcon::fromTheme("dialog-information");
	warningTypeIcon = QIcon::fromTheme("dialog-warning");
	errorTypeIcon = QIcon::fromTheme("dialog-error");
	fatalTypeIcon = QIcon::fromTheme("dialog-error");
	debugTypeIcon = QIcon::fromTheme("dialog-information");
	unknownTypeIcon = QIcon::fromTheme("dialog-question");
}

void LoggingWidget::updateLogMessages() {
	if(logMessagesToAdd.isEmpty())
		return;

	// keep track of the time
	QTime elapsedTime;
	elapsedTime.start();

	// disable sorting to be able to insert multiple items
	tableWidget->setSortingEnabled(false);
	
	QTableWidgetItem* timeItem = NULL;
	QTableWidgetItem* typeItem = NULL;
	QTableWidgetItem* messageItem = NULL;
	while(!logMessagesToAdd.isEmpty() && (elapsedTime.elapsed() < skipInterval)) {
		LogMessage* message = logMessagesToAdd.dequeue();

		int row = tableWidget->rowCount();
		tableWidget->insertRow(row);
	
		timeItem = new QTableWidgetItem(message->time.toString("hh:mm:ss.zzz"));
		messageItem = new QTableWidgetItem(message->message);

		switch(message->type) {
			case InfoMessage:
				typeItem = new QTableWidgetItem(tr("Info"));
				if(!infoTypeIcon.isNull())
					typeItem->setIcon(infoTypeIcon);
				break;
			case WarnMessage:
				typeItem = new QTableWidgetItem(tr("Warn"));
				typeItem->setForeground(Qt::darkYellow);
				if(!warningTypeIcon.isNull())
					typeItem->setIcon(warningTypeIcon);
				messageItem->setForeground(Qt::darkYellow);
				break;
			case ErrorMessage:
				typeItem = new QTableWidgetItem(tr("Error"));
				typeItem->setForeground(Qt::red);
				if(!errorTypeIcon.isNull())
					typeItem->setIcon(errorTypeIcon);
				messageItem->setForeground(Qt::red);
				break;
			case FatalMessage:
				typeItem = new QTableWidgetItem(tr("Fatal"));
				typeItem->setForeground(Qt::darkRed);
				if(!fatalTypeIcon.isNull())
					typeItem->setIcon(fatalTypeIcon);
				messageItem->setForeground(Qt::darkRed);
				break;
			case DebugMessage:
				typeItem = new QTableWidgetItem(tr("Debug"));
				typeItem->setForeground(Qt::darkGray);
				if(!debugTypeIcon.isNull())
					typeItem->setIcon(debugTypeIcon);
				messageItem->setForeground(Qt::darkGray);
				break;
			case UnknownMessage:
				typeItem = new QTableWidgetItem(tr("???"));
				typeItem->setForeground(Qt::darkMagenta);
				if(!unknownTypeIcon.isNull())
					typeItem->setIcon(unknownTypeIcon);
				messageItem->setForeground(Qt::darkMagenta);
				break;
		}

		tableWidget->setItem(row, 0, timeItem);
		tableWidget->setItem(row, 1, typeItem);
		tableWidget->setItem(row, 2, messageItem);

		//resize the height of the row
		tableWidget->resizeRowToContents(row);

		delete message;
	}

	// re-enable sorting again
	tableWidget->setSortingEnabled(true);

	// scroll to last entry
	// TODO: only when the user didn't scroll a position other than the end
	if(scrollToLatestElement) {
		if((tableWidget->currentItem() != NULL) && (messageItem != NULL)) {
			tableWidget->scrollToItem(messageItem);
		}
	}

	// paint the log list
	tableWidget->update();
}
