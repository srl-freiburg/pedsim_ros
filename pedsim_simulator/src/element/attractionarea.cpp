// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/element/attractionarea.h>

// → SGDiCoP
// #include "visual/attractionarearepresentation.h"


AttractionArea::AttractionArea(const QString& nameIn)
	: name(nameIn) {
	// initialize values
// 	representation = new AttractionAreaRepresentation(this);
	size.setWidth(0);
	size.setHeight(0);
	attractionStrength = 0;
}

AttractionArea::~AttractionArea() {
	// clean up
// 	delete representation;
}

QString AttractionArea::getName() const {
	return name;
}

Ped::Tvector AttractionArea::getPosition() const {
	return position;
}

void AttractionArea::setPosition(double xIn, double yIn) {
	position.x = xIn;
	position.y = yIn;

	// inform users
	emit positionChanged(position.x, position.y);
}

void AttractionArea::setPosition(const Ped::Tvector& posIn) {
	position = posIn;

	// inform users
	emit positionChanged(position.x, position.y);
}

void AttractionArea::setx(double xIn) {
	position.x = xIn;

	// inform users
	emit positionChanged(position.x, position.y);
}

void AttractionArea::sety(double yIn) {
	position.y = yIn;

	// inform users
	emit positionChanged(position.x, position.y);
}

QSizeF AttractionArea::getSize() const {
	return size;
}

void AttractionArea::setSize(double widthIn, double heightIn) {
	size.setWidth(widthIn);
	size.setHeight(heightIn);

	// inform users
	emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setSize(const QSizeF& sizeIn) {
	size = sizeIn;

	// inform users
	emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setWidth(double widthIn) {
	size.setWidth(widthIn);

	// inform users
	emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setHeight(double heightIn) {
	size.setHeight(heightIn);

	// inform users
	emit sizeChanged(size.width(), size.height());
}

double AttractionArea::getStrength() const {
	return attractionStrength;
}

void AttractionArea::setStrength(double strengthIn) {
	attractionStrength = strengthIn;

	// inform users
	emit strengthChanged(attractionStrength);
}

QString AttractionArea::toString() const {
	return tr("AttractionArea: '%1' (@%2, %3)")
		.arg(name).arg(position.x).arg(position.y);
}