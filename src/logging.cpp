// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "logging.h"
// -> Qt
#include <QFileInfo>
#include <QTextStream>


LogMessage::LogMessage() 
	: type(InfoMessage) {
	throw "DO NOT USE THIS CONSTRUCTOR!";
}

LogMessage::LogMessage(LogType typeIn, const QTime& timeIn, const QString& messageIn)
	: type(typeIn), time(timeIn), message(messageIn) {
}

//initialize static logger
Logger* Logger::logger = NULL;


Logger& Logger::instance() {
	if(logger == NULL)
		logger = new Logger();

	return (*logger);
}

void Logger::log(LogType typeIn, const QString& messageTextIn) {
	LogMessage message(typeIn, QTime::currentTime(), messageTextIn);
	log(message);
}

void Logger::log(const LogMessage& messageIn) {
	//inform listener
	emit onNewLogMessage(messageIn);
}

FileLogger::FileLogger(const QString& filenameIn) {
	outFile.setFileName(filenameIn);

	if(outFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
		outStream = new QTextStream(&outFile);

		DEBUG_LOG("FileLogger: Writing log output to file: '%1'", QFileInfo(outFile).canonicalFilePath());

		//connect to the logging instance
		qRegisterMetaType<LogMessage>("LogMessage");
		connect(&LOGGER, SIGNAL(onNewLogMessage(const LogMessage&)), this, SLOT(onNewLogMessage(const LogMessage&)));
	}
	else {
		ERROR_LOG("FileLogger: Unable to output open file: '%1'", filenameIn);
	}
}

FileLogger::~FileLogger() {
	delete outStream;
	outFile.close();
}

void FileLogger::onNewLogMessage(LogMessage messageIn) {
	if(outStream == NULL)
		return;

	QString typeString = tr("Unknown");
	switch(messageIn.type) {
		case InfoMessage:
			typeString = tr("Information");
			break;
		case WarnMessage:
			typeString = tr("Warning");
			break;
		case ErrorMessage:
			typeString = tr("Error");
			break;
		case FatalMessage:
			typeString = tr("Fatal");
			break;
		case DebugMessage:
			typeString = tr("Debug");
			break;
		case UnknownMessage:
			typeString = tr("???");
			break;
	}

	*outStream << "[" << messageIn.time.toString("hh:mm:ss.zzz") << "] [" << typeString << "] " << messageIn.message << endl;
	outStream->flush();
}

ConsoleLogger::ConsoleLogger() {
	//connect to the logging instance
	qRegisterMetaType<LogMessage>("LogMessage");
	connect(&LOGGER, SIGNAL(onNewLogMessage(const LogMessage&)), this, SLOT(onNewLogMessage(const LogMessage&)));
}

ConsoleLogger::~ConsoleLogger() {

}

void ConsoleLogger::onNewLogMessage(LogMessage messageIn) {
	std::ostream* out = &std::clog;
	QString typeString(tr("?"));
	switch(messageIn.type) {
		case InfoMessage:
			out = &std::clog;
			typeString = tr("I");
			break;
		case WarnMessage:
			out = &std::clog;
			typeString = tr("W");
			break;
		case ErrorMessage:
			out = &std::cerr;
			typeString = tr("E");
			break;
		case FatalMessage:
			out = &std::cerr;
			typeString = tr("F");
			break;
		case DebugMessage:
			out = &std::clog;
			typeString = tr("D");
			break;
		case UnknownMessage:
			out = &std::clog;
			typeString = tr("?");
			break;
	}
	QString logLine(tr("[%1] [%2] %3\n").arg(messageIn.time.toString("hh:mm:ss.zzz")).arg(typeString).arg(messageIn.message));

	*out << qPrintable(logLine);
}
