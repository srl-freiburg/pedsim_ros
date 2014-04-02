// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _scenarioelement_h_
#define _scenarioelement_h_

// Includes
#include <QObject>


class ScenarioElement : public QObject {
	Q_OBJECT

	// Constructor and Destructor
public:
	ScenarioElement(QObject* parent = nullptr);


	// Signals
signals:
	void selectionChanged(bool selected);


	// Methods
public:
	// → Selection
	virtual void setSelected(bool selectedIn) { emit selectionChanged(selectedIn); };

	virtual QString toString() const = 0;
};

#endif
