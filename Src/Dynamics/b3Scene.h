#ifndef __B3_SCENE_H__
#define __B3_SCENE_H__

#include "..\Common\Memory\b3StackAllocator.h"
#include "..\Common\Memory\b3BlockAllocator.h"
#include "..\Common\b3Time.h"
#include "Contacts\b3ContactGraph.h"

struct b3BodyDef;
class b3Body;
class b3QueryListener;
class b3RayCastListener;
class b3ContactListener;
class b3Draw;

class b3Scene {
public :
	// Use a physics scene to create/destroy (rigid) bodies, execute ray cast and AABB queries.
	b3Scene();
	~b3Scene();

	// Allocates a new rigid body and insert it into the scene.
	b3Body* CreateBody(const b3BodyDef& def);
	
	// Remove a rigid body from the scene and deallocate it from the memory.
	void DestroyBody(b3Body* body);
	
	// Set the gravity direction (usually y-down (0, -1, 0)).
	void SetGravityDirection(const b3Vec3& direction);

	// The contact listener passed will be notified when two body shapes begin/stays/ends
	// touching with each other.
	void SetContactListener(b3ContactListener* listener);

	// The query listener will be notified when two shape AABBs are overlapping.
	// If the listener returns false then the query is stopped immediately.
	// Otherwise, it continues searching for new overlapping shape AABBs.
	void QueryAABB(b3QueryListener* listener, const b3AABB& aabb) const;
	
	// The ray cast listener will be notified when a ray intersects a scene shape.
	// The ray cast output is the intercepted shape, the intersection point, normal, and fraction.
	void RayCast(b3RayCastListener* listener, const b3Vec3& p1, const b3Vec3& p2) const;

	// Call the function below to simulate a physics frame.
	// The function parameter is the time step definition, which must contain 
	// the simulation frequency (dt), the number of contact solver iterations, and 
	// a flag that tells if the bodies should sleep when they're not moving significantly.
	void Step(const b3TimeStep& step);

	// Return the time spend to execute each simulation module of last physics step.
	const b3StepProfile& GetStepProfile() const;

	// Call the function below to debug the physics scene.
	void Draw(const b3Draw* draw, u32 flags) const;
protected :
	enum b3WorldFlags {
		e_bodyAddedFlag = 0x0001,
		e_clearForcesFlag = 0x0002,
	};

	friend class b3Body;

	void ClearForces();
	void Solve(const b3TimeStep& step);
	
	b3StackAllocator m_stackAllocator;
	b3BlockAllocator m_blockAllocator;
	b3ContactGraph m_contactGraph;
	u32 m_flags;
	b3Vec3 m_gravityDir;
	b3Body* m_bodyList;
	u32 m_bodyCount;
	b3StepProfile m_profile;
};

inline const b3StepProfile& b3Scene::GetStepProfile() const {
	return m_profile;
}

inline void b3Scene::SetGravityDirection(const b3Vec3& direction) {
	m_gravityDir = direction;
}

inline void b3Scene::SetContactListener(b3ContactListener* listener) {
	m_contactGraph.m_contactListener = listener;
}

#endif