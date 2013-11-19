// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor
//		http://pedsim.silmaril.org/
//	       _
//	   _  (__/   _ _ _   /    _ / _ ' __/'_    _ /    _ 
//	 _) ()/ /((/(// (-  ()(/ ( /)/ /_) //(//) (/(()()/  
//	                      /                  _/         

// Includes
// → SGDiCoP
#include "sgdicop.h"
// → Qt
#include <QApplication>
#include <QStringList>


/// The programm entry point. Sets up everything, and calls QT exec to start the event loop.
/// \date    2003-12-29
/// \return  whatever app.exec() returns. 
int main(int argc, char** argv) {
	// initialize resources
	Q_INIT_RESOURCE(sgdicop);

	// create a Qt application
	QApplication app(argc, argv);
	
	// set meta-information
	app.setOrganizationName("University Freiburg, Social Robotics Laboratory");
	app.setOrganizationDomain("srl.informatik.uni-freiburg.de");
	app.setApplicationName("SGDiCoP");
	//TODO: add version string method etc.
// 	app.setApplicationVersion(getVersionString());

	
	// copy the options from argc to a QStringList
	QStringList options;
	for(int i = 1; i < argc; i++) {
		QString currentString(argv[i]);
		options.append(currentString);
	}

	// initialize the real application
	SGDiCoP sgdicop;
	sgdicop.initialize(options);
	sgdicop.start();
	
	// run qt application, and return its return value
	return app.exec();
}
