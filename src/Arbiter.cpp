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

#include "box2d-lite/Arbiter.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/World.h"

//Arbiter 헤더파일에 선언했던 변수를 불러옴
bool Arbiter::flag2 = false;

Arbiter::Arbiter(Body* b1, Body* b2)
{
	if (b1 < b2)
	{
		body1 = b1;
		body2 = b2;
	}
	else
	{
		body1 = b2;
		body2 = b1;
	}

	//numContacts = Collide(contacts, body1, body2);
	numContacts = 0; // NEW!
	if (body1->isItExist && body2->isItExist) {
		//printf("meow");
		numContacts = Collide(contacts, body1, body2);
	}


	// flag2에 따른 빙판 
	if(flag2 == true)
	{
		friction = 0;
	}
	else
	{
		friction = sqrtf(body1->friction * body2->friction);
	}
}

void Arbiter::Update(Contact* newContacts, int numNewContacts)
{
	Contact mergedContacts[2];

	for (int i = 0; i < numNewContacts; ++i)
	{
		Contact* cNew = newContacts + i;
		int k = -1;
		for (int j = 0; j < numContacts; ++j)
		{
			Contact* cOld = contacts + j;
			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			Contact* c = mergedContacts + i;
			Contact* cOld = contacts + k;
			*c = *cNew;
			if (World::warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = 0.0f;
				c->Pt = 0.0f;
				c->Pnb = 0.0f;
			}
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts ; ++i)
		contacts[i] = mergedContacts[i];

	numContacts = numNewContacts;
}


void Arbiter::PreStep(float inv_dt)
{
	const float k_allowedPenetration = 0.01f;
	float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;

		Vec2 r1 = c->position - body1->position;
		Vec2 r2 = c->position - body2->position;

		// Precompute normal mass, tangent mass, and bias.
		float rn1 = Dot(r1, c->normal);
		float rn2 = Dot(r2, c->normal);
		float kNormal = body1->invMass + body2->invMass;
		kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
		c->massNormal = 1.0f / kNormal;

		Vec2 tangent = Cross(c->normal, 1.0f);
		float rt1 = Dot(r1, tangent);
		float rt2 = Dot(r2, tangent);
		float kTangent = body1->invMass + body2->invMass;
		kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
		c->massTangent = 1.0f /  kTangent;

		c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->separation + k_allowedPenetration);

		if (World::accumulateImpulses)
		{
			// Apply normal + friction impulse
			Vec2 P = c->Pn * c->normal + c->Pt * tangent;

			body1->velocity -= body1->invMass * P;
			body1->angularVelocity -= body1->invI * Cross(r1, P);

			body2->velocity += body2->invMass * P;
			body2->angularVelocity += body2->invI * Cross(r2, P);
		}
	}
}

//void Arbiter::ApplyImpulse()
void Arbiter::ApplyImpulse(Body** deadBodyStoragePtr, int numStorage)
{
	//printf("debug - ApplyImpulse \n");
	Body* b1 = body1;
	Body* b2 = body2;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;
		c->r1 = c->position - b1->position;
		c->r2 = c->position - b2->position;

		// Relative velocity at contact
		Vec2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

		// Compute normal impulse
		float vn = Dot(dv, c->normal);

		float dPn = c->massNormal * (-vn + c->bias);

		if (World::accumulateImpulses)
		{
			// Clamp the accumulated impulse
			float Pn0 = c->Pn;
			c->Pn = Max(Pn0 + dPn, 0.0f);
			dPn = c->Pn - Pn0;
		}
		else
		{
			dPn = Max(dPn, 0.0f);
		}

		// Apply contact impulse
		Vec2 Pn = dPn * c->normal;

		b1->velocity -= b1->invMass * Pn;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

		b2->velocity += b2->invMass * Pn;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pn);

		// Relative velocity at contact
		dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

		Vec2 tangent = Cross(c->normal, 1.0f);
		float vt = Dot(dv, tangent);
		float dPt = c->massTangent * (-vt);

		if (World::accumulateImpulses)
		{
			// Compute friction impulse
			float maxPt = friction * c->Pn;

			// Clamp friction
			float oldTangentImpulse = c->Pt;
			c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c->Pt - oldTangentImpulse;
		}
		else
		{
			float maxPt = friction * dPn;
			dPt = Clamp(dPt, -maxPt, maxPt);
		}

		// Apply contact impulse
		Vec2 Pt = dPt * tangent;

		b1->velocity -= b1->invMass * Pt;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);

		b2->velocity += b2->invMass * Pt;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
	}
	
	for (int i = 0; i < 2; i++) {
		Body* targetBody[2] = { body1, body2 };

		if (targetBody[i]->isBreakAble == false) continue;
		if (targetBody[i]->impulseLimit < Max(contacts[0].Pn,contacts[1].Pn)) {
			printf("[Debug] OverImpulse detected : %f \n", Max(contacts[0].Pn, contacts[1].Pn));

			//World::KillBody(targetBody[i]);

			for (int k = 0;k < 200;k++) {
				if (deadBodyStoragePtr[k] == NULL) {
					deadBodyStoragePtr[k] = targetBody[i];
					printf("[DEBUG] Body Killed. \n");

					//targetBody[i]->setPosition2(Vec2(2, 2));
					//targetBody[i]->position.Set(2, 2);
					
					//targetBody[i]->isItExist = false;
					targetBody[i]->isItExist = false;

					break;
				}
				if (deadBodyStoragePtr[k] == targetBody[i])
				{
					//printf("[DEBUG] Body Checking...");
					break; // 이미 제거할 목록에 기록되어 있습니다.
				}

				if (k >= numStorage) {
					printf("<!>ERROR<!> Cannot Kill Body Anymore! : deadBodyStoragePtr is full.");
					break;
				}
			}
		}
	}

}
