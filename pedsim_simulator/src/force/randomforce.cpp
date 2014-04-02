// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/randomforce.h>
// → SGDiCoP
#include "config.h"
// #include "logging.h"
#include <pedsim_simulator/rng.h>
#include "scene.h"
// → Qt
#include <QSettings>


RandomForce::RandomForce(Agent* agentIn)
	: Force(agentIn) {
	// initialize values
	setFactor(CONFIG.forceRandom);
	QSettings settings;
	fadingDuration = settings.value("RandomForce/FadingDuration", 1).toDouble();
	nextDeviation = computeNewDeviation();

	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorRandomChanged(double)),
		this, SLOT(onForceFactorChanged(double)));
}

void RandomForce::onForceFactorChanged(double valueIn) {
	setFactor(valueIn);
}

void RandomForce::setFadingTime(double durationIn) {
	// sanity checks
	if(durationIn < 0) {
		ERROR_LOG("Cannot set fading time to invalid value: %1", durationIn);
		return;
	}
	
	fadingDuration = durationIn;
}

double RandomForce::getFadingTime() const {
	return fadingDuration;
}

Ped::Tvector RandomForce::computeNewDeviation() {
	// set up random distributions
	uniform_real_distribution<double> angleDistribution(0, 360);
	double deviationAngle = angleDistribution(RNG());
	normal_distribution<double> distanceDistribution(0, 1);
	double deviationDistance = distanceDistribution(RNG());

	// create deviation from polar coordinates
	Ped::Tvector deviation = Ped::Tvector::fromPolar(Ped::Tangle::fromDegree(deviationAngle), deviationDistance);
	return deviation;
}

Ped::Tvector RandomForce::getForce(Ped::Tvector walkingDirection) {
	// use the current time to compute the fading progress
	double time = SCENE.getTime();
	double progress = fmod(time, fadingDuration);

	// create a new fading goal when necessary
	if(progress < CONFIG.timeStepSize) {
		lastDeviation = nextDeviation;
		nextDeviation = computeNewDeviation();
	}

	// compute the force
	Ped::Tvector force = (1-progress)*lastDeviation + progress*nextDeviation;

	// scale force
	force *= factor;
	
	return force;
}

QString RandomForce::toString() const {
	return tr("RandomForce (fading duration: %1; factor: %2)")
		.arg(fadingDuration)
		.arg(factor);
}
