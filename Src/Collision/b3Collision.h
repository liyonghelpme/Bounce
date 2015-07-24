#ifndef __B3_COLLISION_H__
#define __B3_COLLISION_H__

#include "..\Common\Math\b3Math.h"
#include "b3AABB.h"

inline bool b3TestOverlap(const b3AABB& aabb1, const b3AABB& aabb2) {
	return (aabb1.min.x <= aabb2.max.x) && (aabb1.min.y <= aabb2.max.y) && (aabb1.min.z <= aabb2.max.z) &&
		(aabb1.max.x >= aabb2.min.x) && (aabb1.max.y >= aabb2.min.y) && (aabb1.max.z >= aabb2.min.z);
}

struct b3Pair {
	i32 proxy1;
	i32 proxy2;
};

struct b3RayCastInput {
	b3Vec3 p1;
	b3Vec3 p2;
	r32 maxFraction;
};

struct b3RayCastOutput {
	r32 fraction;
	b3Vec3 normal;
};

#define NULL_EDGE 0xff

struct b3FeaturePair {
	u8 inEdge1;
	u8 outEdge1;
	u8 inEdge2;
	u8 outEdge2;
};

union b3ContactID {
	b3FeaturePair featurePair;
	u32 key;
};

struct b3ContactPoint {
	b3Vec3 position;
	r32 normalImpulse;
	b3Vec3 tangents[B3_MAX_TANGENT_DIRECTIONS];
	r32 tangentImpulse[B3_MAX_TANGENT_DIRECTIONS];
	b3ContactID id;
	bool warmStarted;
};

struct b3Manifold {
	void AddEntry(const b3Vec3& position, r32 distance, b3ContactID id);

	b3Vec3 normal; //A -> B.
	b3ContactPoint points[B3_MAX_MANIFOLD_POINTS];
	r32 distances[B3_MAX_MANIFOLD_POINTS];
	u32 pointCount;
};

inline void b3Manifold::AddEntry(const b3Vec3& position, r32 distance, b3ContactID id) {
	if (pointCount == B3_MAX_MANIFOLD_POINTS) {
		return;
	}
	points[pointCount].position = position;
	distances[pointCount] = distance;
	points[pointCount].id = id;
	++pointCount;
}

#endif