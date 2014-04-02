// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/rng.h>
// → Qt
#include <QSettings>


// Initialize Static Variables
RandomNumberGenerator* RandomNumberGenerator::instance;


RandomNumberGenerator::RandomNumberGenerator() {
	std::random_device randomDevice;
	uint seed = randomDevice();
	randomEngine.seed(seed);
}

RandomNumberGenerator& RandomNumberGenerator::getInstance() {
	if(instance == nullptr)
		instance = new RandomNumberGenerator;
	return *instance;
}

std::default_random_engine& RandomNumberGenerator::operator()() {
	return randomEngine;
}
