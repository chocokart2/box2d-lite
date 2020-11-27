#pragma once
/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <map>
#include "OurMath.h"
#include "OurArbiter.h"

class OurBody;
class OurJoint;

class OurWorld
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurBody;
	friend class OurJoint;
	friend class MainClass;
	friend class OurCollideClass

private:
	std::vector<OurBody*> bodies;
	std::vector<OurJoint*> joints;
	std::map<OurArbiterKey, OurArbiter> arbiters;
	V2 gravity;
	int iterations;
	static bool accumulateImpulses;
	static bool warmStarting;
	static bool positionCorrection;

public:
	OurWorld(V2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

	void Add(OurBody* body);
	void Add(OurJoint* joint);
	void Clear();

	void Step(float dt);

	void BroadPhase();
};

#endif
