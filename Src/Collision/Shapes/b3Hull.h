#ifndef __B3_HULL_H__
#define __B3_HULL_H__

#include "..\..\Common\Math\b3Math.h"

/*****************************************************
The half-edge structure is a edge-centric mesh.
An half-edge based mesh will increase the performance 
of the SAT and it will facilitate later collision 
detection optimizations.
*****************************************************/

#define NULL_FEATURE -1

struct b3Face {
	b3Face() :
		edge(NULL_FEATURE) {
	}
	i32 edge;
};

struct b3HalfEdge {
	b3HalfEdge() :
		prev(NULL_FEATURE),
		next(NULL_FEATURE),
		twin(NULL_FEATURE),
		origin(NULL_FEATURE),
		face(NULL_FEATURE) {
	}
	i32 prev;
	i32 next;
	i32 twin;
	i32 face;
	i32 origin;
};

struct b3HullDef {
	// Use this definition to create convex polyhedron hulls.
	// Thus, a topology defines a convex polygon soup.
	// A face is defined as a index to a face topology.
	// The face topology has the following layout:
	// 1º index: face vertex count.
	// 2º...nº index: face vertex indices.
	// Quick example:
	// faceCount = 2;
	// faces[0] = 0;
	// faces[1] = 5;
	// faceTopology[0] = 4;
	// faceTopology[1] = v1;
	// faceTopology[2] = v2;
	// faceTopology[3] = v3;
	// faceTopology[4] = v4;
	// faceTopology[5] = 3;
	// faceTopology[6] = vx;
	// faceTopology[7] = vy;
	// faceTopology[8] = vz;

	// You should pass a pointer to the array of convex planes
	// as well the vertices, and vertex indices.
	
	const b3Vec3* vertices;
	u32 vertexCount;
	const b3Plane* planes;
	const u32* facesIndices;
	const u32* faceTopology;
	u32 faceCount;
};

struct b3Hull {
	b3Hull() :
		vertCount(0),
		edgeCount(0),
		faceCount(0) {
	}

	const b3Vec3& GetVertex(u32 i) const;
	const b3Plane& GetPlane(u32 i) const;
	const b3Face* GetFace(u32 i) const;
	const b3HalfEdge* GetEdge(u32 i) const;
	
	// Get the supporting vertex in the local space of this hull.
	b3Vec3 GetSupport(const b3Vec3& direction) const;

	// Get the supporting vertex given in the same space of the transform.
	b3Vec3 GetSupport(const b3Vec3& direction, const b3Transform& transform) const;

	// Call the function below to initialize this convex hull.
	void Set(const b3HullDef* def);

	// Validate a given structure.
	void Validate(const b3HalfEdge* halfEdge);
	void Validate(const b3Face* face);
	// Validate this hull.
	void Validate();

	// Intentionally, keep the number of features limited.
	b3Vec3 vertices[B3_MAX_HULL_VERTICES];
	u32 vertCount;

	b3Face faces[B3_MAX_HULL_FACES];
	b3Plane planes[B3_MAX_HULL_FACES]; // Use face count.
	u32 faceCount;

	b3HalfEdge edges[B3_MAX_HULL_HALF_EDGES];
	u32 edgeCount;
};


#endif