// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _rng_h_
#define _rng_h_


// Includes
// → STL
#include <random>


class RandomNumberGenerator {
	// Constructor and Destructor
protected:
	RandomNumberGenerator();


	// Singleton Design Pattern
	#define RNG RandomNumberGenerator::getInstance()
protected:
	static RandomNumberGenerator* instance;
public:
	static RandomNumberGenerator& getInstance();


	// Operators
public:
	std::default_random_engine& operator()();


	// Attributes
public:
	std::default_random_engine randomEngine;
};

#endif
