#ifndef __B3_SAT_H__
#define __B3_SAT_H__

#include "..\Collision\b3Collision.h"

class b3Shape;

struct b3ClipPlane {
	b3Plane plane;
	u8 edgeId;
};

struct b3ClipVertex {
	b3Vec3 position;
	b3FeaturePair featurePair;
};

struct b3ClipPolygon {
	b3ClipVertex vertices[B3_MAX_FACE_VERTICES];
	u32 vertCount;
};

struct b3ClipPlaneVec {
	b3ClipPlane planes[B3_MAX_HULL_EDGES];
	u32 planeCount;
};

struct b3FaceQuery {
	i32 index;
	r32 distance;
};

struct b3EdgeQuery {
	i32 index1;
	i32 index2;
	r32 distance;
};

// SAT collision detection entry point.
void b3HullHullContact(b3Manifold& output, const b3Transform& transform1, const b3Shape* shape1, const b3Transform& transform2, const b3Shape* shape2);

#endif