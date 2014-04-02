// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _attractionarea_h_
#define _attractionarea_h_

// Includes
// → SGDiCoP
#include "scenarioelement.h"
// → PedSim
#include <libpedsim/ped_vector.h>
// → Qt
#include <QSizeF>


// Forward Declarations
// class AttractionAreaRepresentation;


class AttractionArea : public ScenarioElement {
	Q_OBJECT

	// Constructor and Destructor
public:
	AttractionArea(const QString& nameIn);
	virtual ~AttractionArea();


	// Signals
signals:
	void positionChanged(double x, double y);
	void sizeChanged(double width, double height);
	void strengthChanged(double strength);


	// Methods
public:
	QString getName() const;
	// → Position
	virtual Ped::Tvector getPosition() const;
	virtual void setPosition(double xIn, double yIn);
	virtual void setPosition(const Ped::Tvector& posIn);
	virtual void setx(double xIn);
	virtual void sety(double yIn);
	// → Size
	virtual QSizeF getSize() const;
	virtual void setSize(double widthIn, double heightIn);
	virtual void setSize(const QSizeF& sizeIn);
	virtual void setWidth(double widthIn);
	virtual void setHeight(double heightIn);
	// → Attraction
	virtual double getStrength() const;
	virtual void setStrength(double strengthIn);

	// → ScenarioElement Overrides/Overloads
public:
// 	virtual QPointF getVisiblePosition() const;
// 	virtual void setVisiblePosition(const QPointF& positionIn);
	virtual QString toString() const;


	// Attributes
protected:
	const QString name;
	Ped::Tvector position;
	QSizeF size;
	double attractionStrength;

	// → Graphical Representation
// 	AttractionAreaRepresentation* representation;
};

#endif
