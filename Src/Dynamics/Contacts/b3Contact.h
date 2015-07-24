#ifndef __B3_CONTACT_H__
#define __B3_CONTACT_H__

#include "..\..\Collision\b3Collision.h"

class b3Contact;
class b3Shape;
class b3Body;

// The idea is to allow anything to bounce off a inelastic surface (i.e. e == 0).
inline r32 b3MixRestitution(r32 e1, r32 e2)  {
	return b3Max(e1, e2);
}

// The main idea is to drive the restitution to zero. 
inline r32 b3MixFriction(r32 u1, r32 u2) {
	return b3Sqrt(u1 * u2);
}

// The contact edge for the contact graph.
struct b3ContactEdge {
	b3Contact* contact;
	b3Body* other;
	b3ContactEdge* prev;
	b3ContactEdge* next;
};

class b3Contact {
public :
	// Get the fixture that is in contact with the shape B.
	b3Shape* GetShapeA();
	const b3Shape* GetShapeA() const;

	// Get the fixture that is in contact with the shape A.
	b3Shape* GetShapeB();
	const b3Shape* GetShapeB() const;

	// Are the two objects touching?
	bool IsTouching() const;
protected :
	enum b3ContactFlags {
		e_touchingFlag = 0x0001,
		e_islandFlag = 0x0002,
	};

	friend class b3ContactGraph;
	friend class b3ContactSolver;
	friend class b3Scene;
	friend class b3Body;

	b3Shape* m_shapeA;
	b3Shape* m_shapeB;
	r32 m_friction;
	r32 m_restitution;
	u32 m_flags;
	b3Manifold m_manifold;

	b3ContactEdge m_nodeA;
	b3ContactEdge m_nodeB;

	b3Contact* m_prev;
	b3Contact* m_next;
};

inline bool b3Contact::IsTouching() const {
	return (m_flags & e_touchingFlag) != 0;
}

inline b3Shape* b3Contact::GetShapeA() {
	return m_shapeA;
}

inline const b3Shape* b3Contact::GetShapeA() const {
	return m_shapeA;
}

inline b3Shape* b3Contact::GetShapeB() {
	return m_shapeB;
}

inline const b3Shape* b3Contact::GetShapeB() const {
	return m_shapeB;
}

#endif