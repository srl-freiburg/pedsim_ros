// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _scenarioelement_h_
#define _scenarioelement_h_

// Includes
#include <QObject>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>


class ScenarioElement : public QObject {
	Q_OBJECT

	// Constructor and Destructor
public:
	ScenarioElement(QObject* parent = NULL);


	// Signals
signals:
	void selectionChanged(bool selected);


	// Methods
public:
	// → Selection
	void setSelected(bool selectedIn);
	virtual void updateLookOnSelection(bool selectedIn) = 0;

	virtual QString toString() const = 0;


	// Attributes
protected:
};

#endif
