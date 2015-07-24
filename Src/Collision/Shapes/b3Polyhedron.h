#ifndef __B3_POLYHEDRON_H__
#define __B3_POLYHEDRON_H__

#include "b3Shape.h"

struct b3Hull;

class b3Polyhedron : public b3Shape {
public :
	b3Polyhedron();
	~b3Polyhedron();

	virtual b3ShapeType GetType() const { return e_hull; };

	virtual void ComputeMass(b3MassData* massData, r32 density) const;
	virtual void ComputeAabb(b3AABB& output, const b3Transform& transform) const;
	virtual bool RayCast(const b3RayCastInput& input, b3RayCastOutput& output, const b3Transform& transform) const;

	//@warning A hull can be shared therefore a pointer to it is kept instead of a instance.
	const b3Hull* GetHull() const;	
	
	void SetHull(const b3Hull* hull);
protected :
	const b3Hull* m_hull;
};

inline const b3Hull* b3Polyhedron::GetHull() const { return m_hull; }

inline void b3Polyhedron::SetHull(const b3Hull* hull) {
	m_hull = hull;
}

#endif