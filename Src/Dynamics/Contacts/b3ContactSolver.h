#ifndef __B3_CONTACT_SOLVER_H__
#define __B3_CONTACT_SOLVER_H__

#include "..\..\Common\b3Settings.h"
#include "..\..\Common\Math\b3Math.h"

class b3StackAllocator;
class b3Contact;

// This solver Baumgarte method for constraint stabilization.
// It is rapidly executed, but it takes some time to completely solve a constraint.

struct b3VelocityConstraintPoint {
	b3Vec3 rA;
	b3Vec3 rB;
	r32 normalMass;
	r32 tangentMass[B3_MAX_TANGENT_DIRECTIONS];
	r32 normalImpulse;
	r32 tangentImpulse[B3_MAX_TANGENT_DIRECTIONS];
	r32 velocityBias;
	b3Vec3 tangents[B3_MAX_TANGENT_DIRECTIONS];
};

struct b3ContactVelocityConstraint {
	b3VelocityConstraintPoint points[B3_MAX_MANIFOLD_POINTS];
	b3Vec3 normal;
	r32 invMassA;
	r32 invMassB;
	b3Mat33 invIA;
	b3Mat33 invIB;
	r32 friction;
	r32 restitution;
	u32 indexA;
	u32 indexB;
	u32 pointCount;
};

struct b3Velocity {
	b3Vec3 v;
	b3Vec3 w;
};

struct b3Position {
	b3Vec3 x;
	b3Quaternion q;
};

struct b3ContactSolverDef {
	r32 dt;
	u64 dtMicros;
	b3Contact** contacts;
	u32 count;
	b3Position* positions;
	b3Velocity* velocities;
	b3StackAllocator* allocator;
};

class b3ContactSolver {
public :
	b3ContactSolver(const b3ContactSolverDef* def);
	~b3ContactSolver();

	void InitializeVelocityConstraints();
	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();
protected :
	b3StackAllocator* m_allocator;
	b3Position* m_positions;
	b3Velocity* m_velocities;
	b3Contact** m_contacts;
	b3ContactVelocityConstraint* m_velocityConstraints;
	u32 m_count;
	r32 m_invDt;
};

#endif