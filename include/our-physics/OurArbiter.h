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

#ifndef OUR_ARBITER_H
#define OUR_ARBITER_H

#include "OurMath.h"

class OurBody;

union OurFeaturePair
{
	struct Edges
	{
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e;
	int value;
};

class OurContact
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurBody;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class OurCollideClass;

private:
	V2 position;
	V2 normal;
	V2 r1, r2;
	float separation;
	float Pn;	// accumulated normal impulse
	float Pt;	// accumulated tangent impulse
	float Pnb;	// accumulated normal impulse for position bias
	float massNormal, massTangent;
	float bias;
	OurFeaturePair feature;

public:
	OurContact() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}
};


class OurArbiterKey
{
	friend class OurArbiter;
	friend class OurContact;
	friend class OurBody;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class OurCollideClass;

private:
	OurBody* body1;
	OurBody* body2;

public:
	OurArbiterKey(OurBody* b1, OurBody* b2)
	{
		if (b1 < b2)
		{
			body1 = b1; body2 = b2;
		}
		else
		{
			body1 = b2; body2 = b1;
		}
	}

	OurBody* GetBody1() const { return body1; }
	OurBody* GetBody2() const { return body2; }
};

class OurArbiter
{
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurBody;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class OurCollideClass;

public:
	enum { MAX_POINTS = 2 };

private:
	//OurContact contacts[MAX_POINTS]; // 비주얼 스튜디오에 보이지 않음.
	OurContact contacts[2];
	int numContacts;

	OurBody* body1;
	OurBody* body2;

	// Combined friction
	float friction;

	// 상태 저장 변수 추가
	// (빙판)
	static bool flag2;

public:
	OurBody* GetBody1() { return body1; }
	OurBody* GetBody2() { return body2; }

	OurArbiter(OurBody* b1, OurBody* b2);

	void Update(OurContact* contacts, int numContacts);

	void PreStep(float inv_dt);
	void ApplyImpulse();
};

// This is used by std::set
inline bool operator < (const OurArbiterKey& a1, const OurArbiterKey& a2)
{
	if (a1.GetBody1() < a2.GetBody1())
		return true;

	if (a1.GetBody1() == a2.GetBody1() && a1.GetBody2() < a2.GetBody2())
		return true;

	return false;
}

// 아래는 Collide 부분

enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};

struct ClipVertex
{
	ClipVertex() { fp.value = 0; }
	V2 v;
	OurFeaturePair fp;
};

class OurCollideClass {
public:

	static int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
		const V2& normal, float offset, char clipEdge);
	static void ComputeIncidentEdge(ClipVertex c[2], const V2& h, const V2& pos,
		const M22& Rot, const V2& normal);
	static int OurCollide(OurContact* contacts, OurBody* body1, OurBody* body2);
};


#endif
