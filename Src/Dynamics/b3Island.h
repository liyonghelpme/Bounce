#ifndef __B3_ISLAND_H__
#define __B3_ISLAND_H__

#include "..\Common\Math\b3Math.h"

class b3StackAllocator;
class b3Contact;
class b3Body;
struct b3Velocity;
struct b3Position;
struct b3TimeStep;

struct b3IslandDef {
	r32 dt;
	bool allowSleep;
	u32 velocityIterations;
	b3StackAllocator* allocator;
	u32 bodyCapacity;
	u32 contactCapacity;
};

class b3Island {
public :
	b3Island(const b3IslandDef& def);
	~b3Island();

	void Reset();
	void Add(b3Body* b);
	void Add(b3Contact* c);
	void Solve(const b3Vec3& gravity);
protected :
	friend class b3Scene;

	bool allowSleep;
	r32 dt;
	u32 velocityIterations;

	b3Body** bodies;
	u32 bodyCount;
	u32 bodyCapacity;

	b3Contact** contacts;
	u32 contactCount;	
	u32 contactCapacity;

	b3StackAllocator* allocator;
	b3Position* positions;
	b3Velocity* velocities;
};

#endif