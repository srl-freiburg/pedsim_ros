// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _logging_h_
#define _logging_h_

// Includes
// -> STL
#include <iostream>
#include <stdlib.h>
// -> Qt
#include <QFile>
#include <QMetaType>
#include <QTime>


// Forward Declarations
class QTextStream;


// Log message types
enum LogType {
	UnknownMessage,
	InfoMessage,
	WarnMessage,
	ErrorMessage,
	FatalMessage,
	DebugMessage
};


class LogMessage {
public:
	// Constructor
	explicit LogMessage();
	LogMessage(LogType typeIn, const QTime& timeIn, const QString& messageIn);


	// Attributes
	const LogType type;
	const QTime time;
	const QString message;
};


class Logger : public QObject {
	Q_OBJECT

	// Signals
signals:
	void onNewLogMessage(LogMessage messageIn);


	// Methods
public:
	// -> singleton instance getter
	static Logger& instance();

	// -> log message
	void log(LogType typeIn, const QString& messageTextIn);
	void log(const LogMessage& messageIn);


	// Attributes
protected:
	static Logger* logger;
};


class FileLogger : public QObject {
	Q_OBJECT

	// Constructor and destructor
public:
	FileLogger(const QString& filenameIn);
	virtual ~FileLogger();


	// Slots
protected slots:
	// -> log message
	virtual void onNewLogMessage(LogMessage messageIn);


	// Attributes
protected:
	QFile outFile;
	QTextStream* outStream;
};


class ConsoleLogger : public QObject {
	Q_OBJECT

	// Constructor and destructor
public:
	ConsoleLogger();
	virtual ~ConsoleLogger();


	// Slots
protected slots:
	// -> log message
	virtual void onNewLogMessage(LogMessage messageIn);
};


// Helper macros
#define LOGGER					Logger::instance()

// -> Helper for extended logging
inline void LOG_TR_HELP(LogType type, const QString& text) {
	LOGGER.log(type, text);
}
template<typename argType, typename... restType>
inline void LOG_TR_HELP(LogType type, const QString& text, argType arg, restType... restArgs) {
	LOG_TR_HELP(type, text.arg(arg), restArgs...);
}


// -> Debug
template<typename... argTypes>
inline void DEBUG_LOG(const char* text, argTypes... args) {
	LOG_TR_HELP(DebugMessage, QObject::tr(text), args...);
}

// -> Info
template<typename... argTypes>
inline void INFO_LOG(const char* text, argTypes... args) {
	LOG_TR_HELP(InfoMessage, QObject::tr(text), args...);
}

// -> Warning
template<typename... argTypes>
inline void WARN_LOG(const char* text, argTypes... args) {
	LOG_TR_HELP(WarnMessage, QObject::tr(text), args...);
}

// -> Error
template<typename... argTypes>
inline void ERROR_LOG(const char* text, argTypes... args) {
	LOG_TR_HELP(ErrorMessage, QObject::tr(text), args...);
}

// -> Fatal
template<typename... argTypes>
inline void FATAL_LOG(const char* text, argTypes... args) {
	LOG_TR_HELP(FatalMessage, QObject::tr(text), args...);
	sleep(1);
	::exit(EXIT_FAILURE);
}

#endif
