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

#ifndef OUR_JOINT_H
#define OUR_JOINT_H

#include "OurMath.h"

class OurBody;

class OurJoint
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurBody;
	friend class OurWorld;
	friend class MainClass;
	friend class OurCollideClass

private:
	M22 M;
	V2 localAnchor1, localAnchor2;
	V2 r1, r2;
	V2 bias;
	V2 P;		// accumulated impulse
	OurBody* body1;
	OurBody* body2;
	float biasFactor;
	float softness;

public:
	OurJoint() :
		body1(0), body2(0),
		P(0.0f, 0.0f),
		biasFactor(0.2f), softness(0.0f)
	{}

	void Set(OurBody* body1, OurBody* body2, const V2& anchor);

	void PreStep(float inv_dt);
	void ApplyImpulse();
};

#endif