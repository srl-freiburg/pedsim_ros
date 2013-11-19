// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "scenarioelement.h"


ScenarioElement::ScenarioElement(QObject* parent) 
	: QObject(parent) {
}

void ScenarioElement::setSelected(bool selectedIn) {
// 	setSelected(selectedIn);

	// update appearance
	updateLookOnSelection(selectedIn);

	// inform users
	emit selectionChanged(selectedIn);
}
