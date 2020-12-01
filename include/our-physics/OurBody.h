#pragma once
/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef OUR_BODY_H
#define OUR_BODY_H

#include "OurMath.h"

class OurBody
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class OurCollideClass;

private:
	V2 position;
	float rotation;

	V2 velocity;
	float angularVelocity;

	V2 force;
	float torque;

	V2 width;

	float friction;
	float mass, invMass;
	float I, invI;

public:
	OurBody();

	void Set(const V2& w, float m);

	void AddForce(const V2& f)
	{
		force += f;
	}

	// get set func
	V2 getWidth() { return width; }
};

#endif
