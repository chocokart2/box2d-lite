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

#include "OurWorld.h"
#include "OurBody.h"
#include "OurJoint.h"

using std::vector;
using std::map;
using std::pair;

typedef map<OurArbiterKey, OurArbiter>::iterator OurArbIterator;
typedef pair<OurArbiterKey, OurArbiter> OurArbPair;

bool OurWorld::accumulateImpulses = true;
bool OurWorld::warmStarting = true;
bool OurWorld::positionCorrection = true;

void OurWorld::Add(OurBody* body)
{
	bodies.push_back(body);
}

void OurWorld::Add(OurJoint* joint)
{
	joints.push_back(joint);
}

void OurWorld::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void OurWorld::BroadPhase()
{
	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		OurBody* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			OurBody* bj = bodies[j];

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			OurArbiter newArb(bi, bj);
			OurArbiterKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				OurArbIterator iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(OurArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
}

void OurWorld::Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		OurBody* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque;
	}

	// Perform pre-steps.
	for (OurArbIterator arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		for (OurArbIterator arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.ApplyImpulse();
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		OurBody* b = bodies[i];

		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;

		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f;
	}
}
