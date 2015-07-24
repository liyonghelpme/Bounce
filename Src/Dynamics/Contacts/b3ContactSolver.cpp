#include "b3ContactSolver.h"
#include "b3Contact.h"
#include "..\..\Collision\Shapes\b3Shape.h"
#include "..\..\Dynamics\b3Body.h"
#include "..\..\Common\Memory\b3StackAllocator.h"

b3ContactSolver::b3ContactSolver(const b3ContactSolverDef* def) {
	m_allocator = def->allocator;
	m_contacts = def->contacts;
	m_count = def->count;
	m_invDt = def->dt > B3_ZERO ? B3_ONE / def->dt : B3_ZERO;
	m_positions = def->positions;
	m_velocities = def->velocities;

	m_velocityConstraints = (b3ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(b3ContactVelocityConstraint));

	for (u32 i = 0; i < m_count; ++i) {
		b3Contact* c = m_contacts[i];
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;
		
		b3Body* bodyA = c->m_shapeA->GetBody();
		b3Body* bodyB = c->m_shapeB->GetBody();

		u32 pointCount = c->m_manifold.pointCount;

		b3Assert(pointCount > 0);

		vc->normal = c->m_manifold.normal;
		vc->invMassA = bodyA->m_invMass;
		vc->invMassB = bodyB->m_invMass;
		vc->invIA = bodyA->m_invWorldInertia;
		vc->invIB = bodyB->m_invWorldInertia;
		vc->friction = c->m_friction;
		vc->restitution = c->m_restitution;
		vc->indexA = bodyA->m_islandID;
		vc->indexB = bodyB->m_islandID;
		vc->pointCount = pointCount;

		for (u32 j = 0; j < pointCount; ++j) {
			b3ContactPoint* cp = c->m_manifold.points + j;
			b3VelocityConstraintPoint* vcp = vc->points + j;

			// Setup warm start.
			vcp->normalImpulse = cp->normalImpulse;
			vcp->tangentImpulse[0] = cp->tangentImpulse[0];
			vcp->tangentImpulse[1] = cp->tangentImpulse[1];
			vcp->tangents[0] = cp->tangents[0];
			vcp->tangents[1] = cp->tangents[1];
		}
	}
}

b3ContactSolver::~b3ContactSolver() {
	m_allocator->Free(m_velocityConstraints);
}

void b3ContactSolver::InitializeVelocityConstraints() {
	for (u32 i = 0; i < m_count; ++i) {
		b3Contact* c = m_contacts[i];
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;
		
		r32 mA = vc->invMassA;
		r32 mB = vc->invMassB;
		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;
		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 pointCount = c->m_manifold.pointCount;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 xA = m_positions[indexA].x;
		b3Quaternion qA = m_positions[indexA].q;

		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;
		b3Vec3 xB = m_positions[indexB].x;
		b3Quaternion qB = m_positions[indexB].q;

		for (u32 j = 0; j < pointCount; ++j) {
			b3ContactPoint* cp = c->m_manifold.points + j;
			b3VelocityConstraintPoint* vcp = vc->points + j;

			vcp->rA = cp->position - xA;
			vcp->rB = cp->position - xB;

			// Compute tangent mass.
			b3Vec3 rt1A = b3Cross(vcp->rA, vcp->tangents[0]);
			b3Vec3 rt1B = b3Cross(vcp->rB, vcp->tangents[0]);

			b3Vec3 rt2A = b3Cross(vcp->rA, vcp->tangents[1]);
			b3Vec3 rt2B = b3Cross(vcp->rB, vcp->tangents[1]);

			r32 kTangent1 = mA + mB + b3Dot(rt1A, iA * rt1A) + b3Dot(rt1B, iB * rt1B);
			r32 kTangent2 = mA + mB + b3Dot(rt2A, iA * rt2A) + b3Dot(rt2B, iB * rt2B);

			vcp->tangentMass[0] = B3_ONE / kTangent1;
			vcp->tangentMass[1] = B3_ONE / kTangent2;

			// Compute normal mass.
			b3Vec3 rnA = b3Cross(vcp->rA, vc->normal);
			b3Vec3 rnB = b3Cross(vcp->rB, vc->normal);

			r32 kNormal = mA + mB + b3Dot(rnA, iA * rnA) + b3Dot(rnB, iB * rnB);
			vcp->normalMass = B3_ONE / kNormal;

			vcp->velocityBias = -B3_BAUMGARTE * m_invDt * b3Min(B3_ZERO, c->m_manifold.distances[j] + B3_SLOP);

			// Add restitution in the velocity constraint.			
			r32 vn = b3Dot(vB + b3Cross(wB, vcp->rB) - vA - b3Cross(wA, vcp->rA), vc->normal);
			if (vn < -B3_ONE) {
				vcp->velocityBias += -(c->m_restitution) * vn;
			}
		}
	}
}

void b3ContactSolver::WarmStart() {
	// Warm start.
	for (u32 i = 0; i < m_count; ++i) {
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		b3Vec3 normal = vc->normal;
		r32 mA = vc->invMassA;
		r32 mB = vc->invMassB;
		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;
		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 pointCount = vc->pointCount;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;

		for (u32 j = 0; j < pointCount; ++j) {
			b3VelocityConstraintPoint* vcp = vc->points + j;
			b3Vec3 P = vcp->normalImpulse * normal + vcp->tangentImpulse[0] * vcp->tangents[0] + vcp->tangentImpulse[1] * vcp->tangents[1];
			vA -= mA * P;
			wA -= iA * b3Cross(vcp->rA, P);
			vB += mB * P;
			wB += iB * b3Cross(vcp->rB, P);
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b3ContactSolver::SolveVelocityConstraints() {
	for (u32 i = 0; i < m_count; ++i) {
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		b3Vec3 normal = vc->normal;
		r32 mA = vc->invMassA;
		r32 mB = vc->invMassB;
		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;
		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 pointCount = vc->pointCount;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;

		for (u32 j = 0; j < pointCount; ++j) {
			b3VelocityConstraintPoint* vcp = vc->points + j;

			// Relative velocity at contact.
			{
				b3Vec3 dv = vB + b3Cross(wB, vcp->rB) - vA - b3Cross(wA, vcp->rA);

				// Compute normal impulse.
				r32 vn = b3Dot(dv, normal);
				r32 lambda = vcp->normalMass * (-vn + vcp->velocityBias);
				
				r32 newImpulse = b3Max(vcp->normalImpulse + lambda, B3_ZERO);
				lambda = newImpulse - vcp->normalImpulse;
				vcp->normalImpulse = newImpulse;

				b3Vec3 P = lambda * normal;

				vA -= mA * P;
				wA -= iA * b3Cross(vcp->rA, P);

				vB += mB * P;
				wB += iB * b3Cross(vcp->rB, P);
			}

			for (u32 k = 0; k < 2; ++k) {
				b3Vec3 dv = vB + b3Cross(wB, vcp->rB) - vA - b3Cross(wA, vcp->rA);

				// Compute tangential impulse.
				r32 vt = b3Dot(dv, vcp->tangents[k]);
				r32 lambda = vcp->tangentMass[k] * -vt;

				r32 maxFriction = vc->friction * vcp->normalImpulse;
				r32 newImpulse = b3Clamp(vcp->tangentImpulse[k] + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - vcp->tangentImpulse[k];
				vcp->tangentImpulse[k] = newImpulse;

				b3Vec3 P = lambda * vcp->tangents[k];

				vA -= mA * P;
				wA -= iA * b3Cross(vcp->rA, P);

				vB += mB * P;
				wB += iB * b3Cross(vcp->rB, P);
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b3ContactSolver::StoreImpulses() {
	for (u32 i = 0; i < m_count; ++i) {
		b3Contact* c = m_contacts[i];
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		for (u32 j = 0; j < vc->pointCount; ++j) {
			b3VelocityConstraintPoint* vcp = vc->points + j;
			b3ContactPoint* cp = c->m_manifold.points + j;
			cp->normalImpulse = vcp->normalImpulse;
			cp->tangentImpulse[0] = vcp->tangentImpulse[0];
			cp->tangentImpulse[1] = vcp->tangentImpulse[1];
		}
	}
}