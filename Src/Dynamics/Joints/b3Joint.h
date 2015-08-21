/*
******************************************************************************
   Copyright (c) 2015 Irlan Robson http://www.irlanengine.wordpress.com

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
	 claim that you wrote the original software. If you use this software
	 in a product, an acknowledgment in the product documentation would be
	 appreciated but is not required.
   2. Altered source versions must be plainly marked as such, and must not
	 be misrepresented as being the original software.
   3. This notice may not be removed or altered from any source distribution.
*******************************************************************************
*/

#ifndef __B3_JOINT_H__
#define __B3_JOINT_H__

// @note Joints aren't currently available in bounce.
// The following joint class is from an old physics engine.
// I've derivated myself using Box2D (Lite) as reference
// and some constraint equation and algebra of course.

#include "..\b3Body.h"

class b3Joint {
public :
	b3Vec3 localA;
	b3Vec3 localB;

	b3Vec3 worldA;
	b3Vec3 worldB;

	b3Body* bodyA;
	b3Body* bodyB;

	b3Vec3 rA;
	b3Vec3 rB;

	b3Mat33 M; // Effective mass matrix.
	b3Vec3 velocityBias; // The velocity constraint bias.
	b3Vec3 OP; // Accumulated (old) impulse.

	void Set(b3Body* A, b3Body* B, const b3Vec3& position) {
		bodyA = A;
		bodyB = B;
		localA = b3MulT(bodyA->m_transform, position);
		localB = b3MulT(bodyB->m_transform, position);
	}

	void InitializeVelocityConstraint(float invDt) {
		worldA = bodyA->m_transform * localA;
		worldB = bodyB->m_transform * localB;

		rA = bodyA->m_worldCenter - worldA;
		rB = bodyB->m_worldCenter - worldB;

		b3Mat33 MI;
		MI.x.x = MI.y.y = MI.z.z = bodyA->m_invMass + bodyB->m_invMass;

		b3Mat33 RA;
		RA = b3SkewSymmetric(rA);
		
		b3Mat33 RB;
		RB = b3SkewSymmetric(rB);

		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RBT = b3Transpose(RB);

		b3Mat33 K = MI + (RA * bodyA->m_invWorldInertia * RAT) + (RB * bodyB->m_invWorldInertia * RBT);
		M = b3Inverse(K);

		b3Vec3 distance = worldB - worldA;
		velocityBias = -B3_BAUMGARTE * invDt * distance;

		// Warm start.
		bodyA->ApplyLinearImpulse(OP, worldA, true);
		bodyB->ApplyLinearImpulse(OP, worldB, true);
	}

	void SolveVelocityConstraint() {
		b3Vec3 dv = bodyB->m_linearVelocity + b3Cross(bodyB->m_angularVelocity, rB) - bodyA->m_linearVelocity - b3Cross(bodyA->m_angularVelocity, rA);
		
		b3Vec3 NP = M * (-dv + velocityBias);
		b3Vec3 P = NP - OP;
		OP += P;

		bodyA->ApplyLinearImpulse(P, worldA, true);
		bodyB->ApplyLinearImpulse(P, worldB, true);
	}
};

#endif
