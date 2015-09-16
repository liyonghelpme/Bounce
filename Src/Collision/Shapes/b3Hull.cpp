/*
* Copyright (c) 2015-2015 Irlan Robson http://www.irlans.wordpress.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "b3Hull.h"
#include <map>

const b3Vec3& b3Hull::GetVertex(u32 i) const { return vertices[i]; }

const b3Plane& b3Hull::GetPlane(u32 i) const { return planes[i]; }

const b3Face* b3Hull::GetFace(u32 i) const { return faces + i; }

const b3HalfEdge* b3Hull::GetEdge(u32 i) const { return edges + i; }

b3Vec3 b3Hull::GetSupport(const b3Vec3& direction) const {
	u32 index = 0;
	r32 max = b3Dot(direction, vertices[index]);
	for (u32 i = 1; i < vertCount; ++i) {
		r32 dot = b3Dot(direction, vertices[i]);
		if (dot > max) {
			index = i;
			max = dot;
		}
	}
	return vertices[index];
}

b3Vec3 b3Hull::GetSupport(const b3Vec3& direction, const b3Transform& transform) const {
	u32 index = 0;
	r32 max = b3Dot(direction, transform * vertices[index]);
	for (u32 i = 1; i < vertCount; ++i) {
		r32 dot = b3Dot(direction, transform * vertices[i]);
		if (dot > max) {
			index = i;
			max = dot;
		}
	}
	return transform * vertices[index];
}

void b3Hull::Set(const b3HullDef* def) {
	typedef std::pair<i32, i32> VertexPair;
	typedef std::map< std::pair<i32, i32>, u32> EdgeHalfEdgeMap;

	b3Assert(vertCount == 0);
	b3Assert(edgeCount == 0);
	b3Assert(faceCount == 0);

	b3Assert(def->faceCount < B3_MAX_HULL_FACES);
	b3Assert(def->vertexCount < B3_MAX_HULL_VERTICES);

	vertCount = def->vertexCount;
	::memcpy(&vertices[0], def->vertices, vertCount * sizeof(b3Vec3));

	::memcpy(&planes[0], def->planes, def->faceCount * sizeof(b3Plane));

	EdgeHalfEdgeMap edgeMap;

	for (u32 i = 0; i < def->faceCount; ++i) {
		u32 faceIndex = def->facesIndices[i];
		u32 vertCount = def->faceTopology[faceIndex];

		b3Assert(vertCount < B3_MAX_FACE_VERTICES);

		const u32* verts = &def->faceTopology[faceIndex + 1];

		u32 faceHalfEdges[B3_MAX_HULL_HALF_EDGES];
		u32 faceHalfEdgeCount = 0;
		for (u32 j = 0; j < vertCount; ++j) {
			u32 v1 = verts[j];
			u32 v2 = j + 1  < vertCount ? verts[j + 1] : verts[0];

			i32 e12 = NULL_FEATURE;
			i32 e21 = NULL_FEATURE;

			const auto edgeIter = edgeMap.find(VertexPair(v1, v2));
			if (edgeIter != edgeMap.end()) {
				// Edge found.
				// Grab indices.
				e12 = edgeIter->second;
				e21 = edges[e12].twin;
			}
			else {
				// Edge not found. 
				// Allocate a full edge.
				e12 = edgeCount++;
				e21 = edgeCount++;

				edges[e12].twin = e21;
				edges[e21].twin = e12;

				edgeMap[VertexPair(v1, v2)] = e12;
				edgeMap[VertexPair(v2, v1)] = e21;
			}

			b3HalfEdge& edge12 = edges[e12];
			b3HalfEdge& edge21 = edges[e21];

			// Link outgoing vertex of the edge.
			if (edge12.origin == NULL_FEATURE) {
				edge12.origin = v1;
			}
			// Link incoming vertex of the edge.
			if (edge21.origin == NULL_FEATURE) {
				edge21.origin = v2;
			}

			// Link adjacent face to edge.
			if (edge12.face == NULL_FEATURE) {
				edge12.face = faceCount;
			}

			if (faces[faceCount].edge == NULL_FEATURE) {
				faces[faceCount].edge = e12;
			}

			b3Assert(edgeCount < B3_MAX_HULL_HALF_EDGES);

			faceHalfEdges[faceHalfEdgeCount++] = e12;
		}

		// Link the half-edges of the current face.
		for (u32 k = 0; k < faceHalfEdgeCount; ++k) {
			u32 e1 = faceHalfEdges[k];
			u32 e2 = k + 1 < faceHalfEdgeCount ? faceHalfEdges[k + 1] : faceHalfEdges[0];
			edges[e1].next = e2;
			edges[e2].prev = e1;
		}

		++faceCount;
	}

	Validate();
}

void b3Hull::Validate(const b3HalfEdge* edge) {
	b3Assert(edgeCount > 0);

	const b3HalfEdge* twin = edges + edge->twin;
	i32 edgeIndex = edge - edges;

	b3Assert(twin->twin == edgeIndex);
	b3Assert(edges[edge->prev].next == edgeIndex);
	b3Assert(b3Abs(edge->twin - edgeIndex) == 1);
	b3Assert(edge->origin != twin->origin);

	i32 count = 0;
	const b3HalfEdge* start = edge;
	do {
		const b3HalfEdge* next = edges + edge->next;
		const b3HalfEdge* twin = edges + next->twin;
		edge = twin;
		b3Assert(++count < B3_MAX_HULL_HALF_EDGES);
	} while (edge != start);
}

void b3Hull::Validate(const b3Face* face) {
	b3Assert(edgeCount > 0);
	b3Assert(faceCount > 0);
	
	Validate(edges + face->edge);
}

void b3Hull::Validate() {
	b3Assert(faceCount > 0);
	b3Assert(edgeCount > 0);

	for (u32 i = 0; i < faceCount; ++i) {
		Validate(faces + i);
	}
}
