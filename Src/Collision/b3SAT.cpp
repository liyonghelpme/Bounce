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

#include "b3Sat.h"
#include "Shapes\b3Polyhedron.h"
#include "Shapes\b3Hull.h"

/*
*****************************************************************
References: Dirk Gregorius GDC 2013 Presentation.
A fast implementation of the Separating Axis Test (SAT) using the
Gauss-Map optimization to skip edge pairs that don't build a face 
on the Minkowski Difference (edges that aren't supporting edges).
*****************************************************************
*/

// Query minimum separation distance and axis of the first hull planes.
void b3QueryFaceDirections(b3FaceQuery& out, const b3Transform& transform1, const b3Hull* hull1, const b3Transform& transform2, const b3Hull* hull2) {
	// Perform computations in the local space of the second hull.
	b3Transform transform = b3MulT(transform2, transform1);

	out.distance = -B3_MAX_FLOAT;
	out.index = -1;

	for (u32 i = 0; i < hull1->faceCount; ++i) {
		b3Plane plane = transform * hull1->GetPlane(i);
		b3Vec3 support = hull2->GetSupport(-plane.normal);
		r32 distance = b3Distance(plane, support);

		if (distance > out.distance) {
			out.distance = distance;
			out.index = i;
		}
	}
}

bool b3IsSeparationAxis(const b3Transform& transform, const b3Hull* hull1, const b3Hull* hull2, i32 axis1) {
	b3Plane plane = transform * hull1->GetPlane(axis1);
	b3Vec3 support = hull2->GetSupport(-plane.normal);
	r32 distance = b3Distance(plane, support);
	
	return distance > B3_ZERO ? true : false;
}

// If an edge pair doesn't build a face on the MD then it isn't a supporting edge.
bool b3IsMinkowskiFace(const b3Vec3& A, const b3Vec3& B, const b3Vec3& B_x_A, const b3Vec3& C, const b3Vec3& D, const b3Vec3& D_x_C) {
	// Test if arcs AB and CD intersect on the unit sphere 
	r32 CBA = b3Dot(C, B_x_A);
	r32 DBA = b3Dot(D, B_x_A);
	r32 ADC = b3Dot(A, D_x_C);
	r32 BDC = b3Dot(B, D_x_C);

	return CBA * DBA < B3_ZERO && ADC * BDC < B3_ZERO && CBA * BDC > B3_ZERO;
}

r32 b3Project(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C1) {
	// The given edge pair must create a face on the MD.

	// Compute search direction.
	b3Vec3 E1_x_E2 = b3Cross(E1, E2);

	// Skip if the edges are significantly parallel to each other.
	const r32 kTol = r32(0.005);
	r32 L = b3Len(E1_x_E2);
	if (L < kTol * b3Sqrt(b3LenSq(E1) * b3LenSq(E2))) {
		return -B3_MAX_FLOAT;
	}

	// Assure the normal points from hull1 to hull2.
	b3Vec3 N = (B3_ONE / L) * E1_x_E2;
	if (b3Dot(N, P1 - C1) < B3_ZERO) {
		N = -N;
	}

	// Return the signed distance.
	return b3Dot(N, P2 - P1);
}

void b3QueryEdgeDirections(b3EdgeQuery& out, const b3Transform& transform1, const b3Hull* hull1, const b3Transform& transform2, const b3Hull* hull2) {
	// Query minimum separation distance and axis of the first hull planes.
	
	// Perform computations in the local space of the second hull.
	b3Transform transform = b3MulT(transform2, transform1);
	b3Vec3 C1 = transform.translation /* * hull1.centroid*/;

	out.distance = -B3_MAX_FLOAT;
	out.index1 = -1;
	out.index2 = -1;
	
	for (u32 i = 0; i < hull1->edgeCount; i += 2) {
		const b3HalfEdge* edge1 = hull1->GetEdge(i);
		const b3HalfEdge* twin1 = hull1->GetEdge(i + 1);

		b3Assert(edge1->twin == i + 1 && twin1->twin == i);

		b3Vec3 P1 = transform * hull1->GetVertex(edge1->origin);
		b3Vec3 Q1 = transform * hull1->GetVertex(twin1->origin);
		b3Vec3 E1 = Q1 - P1;

		b3Vec3 U1 = transform.rotation * hull1->GetPlane(edge1->face).normal;
		b3Vec3 V1 = transform.rotation * hull1->GetPlane(twin1->face).normal;

		for (u32 j = 0; j < hull2->edgeCount; j += 2) {
			const b3HalfEdge* edge2 = hull2->GetEdge(j);
			const b3HalfEdge* twin2 = hull2->GetEdge(j + 1);

			b3Assert(edge2->twin == j + 1 && twin2->twin == j);

			b3Vec3 P2 = hull2->GetVertex(edge2->origin);
			b3Vec3 Q2 = hull2->GetVertex(twin2->origin);
			b3Vec3 E2 = Q2 - P2;

			b3Vec3 U2 = hull2->GetPlane(edge2->face).normal;
			b3Vec3 V2 = hull2->GetPlane(twin2->face).normal;

			if ( b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2) ) {
				r32 distance = b3Project(P1, E1, P2, E2, C1);
				if (distance > out.distance) {
					out.index1 = i;
					out.index2 = j;
					out.distance = distance;
				}
			}
		}
	}
}

void b3ClosestPointsSegmentSegment(const b3Vec3& P1, const b3Vec3& Q1, const b3Vec3& P2, const b3Vec3& Q2, b3Vec3 *C1, b3Vec3 *C2) {
	b3Vec3 P2P1 = P1 - P2;
	
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 E2 = Q2 - P2;

	r32 D1 = b3LenSq(E1);
	r32 D2 = b3LenSq(E2);
	
	r32 D12 = b3Dot(E1, E2);
	r32 DE1P1 = b3Dot(E1, P2P1);
	r32 DE2P1 = b3Dot(E2, P2P1);

	r32 DNM = D1 * D2 - D12 * D12;

	// Get the two fractions.
	r32 F1 = (D12 * DE2P1 - DE1P1 * D2) / DNM;
	r32 F2 = (D12 * F1 + DE2P1) / D2;

	*C1 = P1 + F1 * E1;
	*C2 = P2 + F2 * E2;
}

void b3CreateEdgeContact(b3Manifold& output, const b3EdgeQuery& input, const b3Transform& transform1, const b3Hull* hull1, const b3Transform& transform2, const b3Hull* hull2) {
	// Create an edge contact if the minimum separation distance comes from a
	// edge combination.
	const b3HalfEdge* edge1 = hull1->GetEdge(input.index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(input.index1 + 1);

	b3Vec3 P1 = transform1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = transform1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;

	const b3HalfEdge* edge2 = hull2->GetEdge(input.index2);
	const b3HalfEdge* twin2 = hull2->GetEdge(input.index2 + 1);

	b3Vec3 P2 = transform2 * hull2->GetVertex(edge2->origin);
	b3Vec3 Q2 = transform2 * hull2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;

	b3Vec3 normal = b3Normalize(b3Cross(Q1 - P1, Q2 - P2));

	b3Vec3 C2C1 = transform2.translation - transform1.translation;

	b3ContactID id;

	if (b3Dot(normal, C2C1) < B3_ZERO) {
		// Flip
		output.normal = -normal;
		id.featurePair.inEdge1 = input.index2;
		id.featurePair.outEdge1 = input.index2 + 1;

		id.featurePair.inEdge2 = input.index1 + 1;
		id.featurePair.outEdge2 = input.index1;
	}
	else {
		output.normal = normal;

		id.featurePair.inEdge1 = input.index1;
		id.featurePair.outEdge1 = input.index1 + 1;

		id.featurePair.inEdge2 = input.index2 + 1;
		id.featurePair.outEdge2 = input.index2;
	}

	// Compute the closest points between the two edges.
	b3Vec3 C1, C2;
	b3ClosestPointsSegmentSegment(P1, Q1, P2, Q2, &C1, &C2);
	// Use the point between them.
	b3Vec3 position = B3_HALF * (C1 + C2);
	output.AddEntry(position, input.distance, id);
}

void b3Clip(const b3ClipPolygon& input, const b3ClipPlane& plane, b3ClipPolygon& output) {
	/*
	* Perform the Sutherland-Hodgman polygon clipping. Since all side planes are pointing 
	outwards the points that are *behind* the plane are kept.
	*/

	b3Assert(output.vertCount == 0);
	b3Assert(input.vertCount >= 3);

	b3ClipVertex Vertex1 = input.vertices[input.vertCount - 1];
	r32 Distance1 = b3Distance(plane.plane, Vertex1.position);

	for (u32 Index = 0; Index < input.vertCount; ++Index) {
		b3ClipVertex Vertex2 = input.vertices[Index];
		r32 Distance2 = b3Distance(plane.plane, Vertex2.position);

		if (Distance1 <= B3_ZERO && Distance2 <= B3_ZERO) {
			// Both vertices are behind or lying on the plane -> keep vertex2
			output.vertices[output.vertCount] = Vertex2;
			++output.vertCount;
		}
		else if (Distance1 <= B3_ZERO && Distance2 > B3_ZERO) {
			// Vertex1 is behind the plane, vertex2 is in front -> intersection point
			r32 Fraction = Distance1 / (Distance1 - Distance2);
			b3Vec3 Position = Vertex1.position + Fraction * (Vertex2.position - Vertex1.position);

			// Keep intersection point 
			b3ClipVertex Vertex;
			Vertex.position = Position;
			Vertex.featurePair.inEdge1 = NULL_EDGE;
			Vertex.featurePair.inEdge2 = Vertex1.featurePair.outEdge2;
			Vertex.featurePair.outEdge1 = plane.edgeId;
			Vertex.featurePair.outEdge2 = NULL_EDGE;

			output.vertices[output.vertCount] = Vertex;
			++output.vertCount;
		}
		else if (Distance2 <= B3_ZERO && Distance1 > B3_ZERO)	{
			// Vertex2 is behind of the plane, vertex1 is in front -> intersection point
			r32 Fraction = Distance1 / (Distance1 - Distance2);
			b3Vec3 Position = Vertex1.position + Fraction * (Vertex2.position - Vertex1.position);

			// Keep intersection point 
			b3ClipVertex Vertex;
			Vertex.position = Position;
			Vertex.featurePair.inEdge1 = plane.edgeId;
			Vertex.featurePair.outEdge1 = NULL_EDGE;
			Vertex.featurePair.inEdge2 = NULL_EDGE;
			Vertex.featurePair.outEdge2 = Vertex1.featurePair.outEdge2;

			output.vertices[output.vertCount] = Vertex;
			++output.vertCount;

			// And also keep vertex2
			output.vertices[output.vertCount] = Vertex2;
			++output.vertCount;
		}

		// Keep vertex2 as starting vertex for next edge
		Vertex1 = Vertex2;
		Distance1 = Distance2;
	}
}

void b3ComputeReferenceFaceSidePlanes(b3ClipPlaneVec& output, const b3Plane& facePlane, u32 faceIndex, const b3Transform& transform, const b3Hull* hull) {
	const b3HalfEdge* start = hull->GetEdge( hull->GetFace(faceIndex)->edge );
	const b3HalfEdge* current = start;
	do {
		const b3HalfEdge* twin = hull->GetEdge(current->twin);

		b3Vec3 P = transform * hull->GetVertex(current->origin);
		b3Vec3 Q = transform * hull->GetVertex(twin->origin);
		
		b3Assert(output.planeCount < B3_MAX_HULL_EDGES);
		
		b3ClipPlane clipPlane;
		clipPlane.edgeId = twin->twin; //edge ID.
		clipPlane.plane.normal = b3Normalize(b3Cross(Q - P, facePlane.normal));
		clipPlane.plane.offset = b3Dot(clipPlane.plane.normal, P);
		
		output.planes[output.planeCount] = clipPlane;
		++output.planeCount;

		current = hull->GetEdge(current->next);
	} while (current != start);
}

void b3ComputeIncidentFacePolygon(b3ClipPolygon& output, const b3Plane& facePlane, const b3Transform& transform, const b3Hull* hull) {
	u32 index = 0;
	r32 min = b3Dot(facePlane.normal, transform.rotation * hull->GetPlane(index).normal);
	for (u32 i = 1; i < hull->faceCount; ++i) {
		r32 dot = b3Dot(facePlane.normal, transform.rotation * hull->GetPlane(i).normal);
		if (dot < min) {
			min = dot;
			index = i;
		}
	}

	// Clip this polygon against all planes here.

	const b3HalfEdge* start = hull->GetEdge(hull->GetFace(index)->edge);
	const b3HalfEdge* current = start;
	do {
		const b3HalfEdge* twin = hull->GetEdge(current->twin);

		b3Vec3 P = transform * hull->GetVertex(current->origin);

		b3Assert(output.vertCount <= B3_MAX_FACE_VERTICES);

		//@todo I'm not completely sure if the contact IDs are correct.
		b3ClipVertex clipVertex;
		clipVertex.featurePair.inEdge1 = NULL_EDGE;
		clipVertex.featurePair.outEdge1 = NULL_EDGE;
		clipVertex.featurePair.inEdge2 = current->next;
		clipVertex.featurePair.outEdge2 = twin->twin;
		clipVertex.position = P;
		
		output.vertices[output.vertCount] = clipVertex;
		++output.vertCount;

		current = hull->GetEdge(current->next);
	} while (current != start);
}

void b3CreateFaceContact(b3Manifold& output, const b3FaceQuery& input, const b3Transform& transform1, const b3Hull* hull1, const b3Transform& transform2, const b3Hull* hull2, bool flipNormal) {
	b3Plane referencePlane = transform1 * hull1->GetPlane(input.index);

	// Create reference face side planes.
	b3ClipPlaneVec planes;
	planes.planeCount = 0;
	b3ComputeReferenceFaceSidePlanes(planes, referencePlane, input.index, transform1, hull1);

	// Create incident face polygon.
	b3ClipPolygon incidentPolygon;
	incidentPolygon.vertCount = 0;
	b3ComputeIncidentFacePolygon(incidentPolygon, referencePlane, transform2, hull2);

	// Clip incident face polygon against the reference face side planes.
	b3ClipPolygon clippedPolygon = incidentPolygon;
	for (u32 i = 0; i < planes.planeCount; ++i) {
		b3ClipPolygon outputPolygon;
		outputPolygon.vertCount = 0;
		b3Clip(clippedPolygon, planes.planes[i], outputPolygon);
		if (outputPolygon.vertCount == 0) {
			return;
		}
		clippedPolygon = outputPolygon;
	}

	// Get all contact points below reference face.
	for (u32 i = 0; i < clippedPolygon.vertCount; ++i) {
		b3ClipVertex vertex = clippedPolygon.vertices[i];
		r32 distance = b3Distance(referencePlane, vertex.position);

		if (distance <= B3_ZERO) {
			// Below reference plane -> position constraint violated.
			b3ContactID id;
			id.featurePair = vertex.featurePair;
			
			if (flipNormal) {
				output.normal = -referencePlane.normal;
				b3Swap(id.featurePair.inEdge1, id.featurePair.inEdge2);
				b3Swap(id.featurePair.outEdge1, id.featurePair.outEdge2);
			}
			else {
				output.normal = referencePlane.normal;
			}

			// Project clipped point onto reference plane.
			b3Vec3 position = b3ClosestPoint(referencePlane, vertex.position);
			// Add point and distance to the plane to the manifold.
			output.AddEntry(position, distance, id);
		}
	}
}

void b3HullHullContact(b3Manifold& output, const b3Transform& transform1, const b3Shape* shape1, const b3Transform& transform2, const b3Shape* shape2) {
	const b3Hull* hull1 = ((b3Polyhedron*)shape1)->GetHull();
	const b3Hull* hull2 = ((b3Polyhedron*)shape2)->GetHull();
	/*
	if (output.cache.type != -1) {

		if (output.cache.type == 1) {
			if (output.cache.shape == 1) {
				b3Transform transform = b3MulT(transform2, transform1);
				if (b3IsSeparationAxis(transform, hull1, hull2, output.cache.face)) {
					return;
				}
			}
			else {
				b3Transform transform = b3MulT(transform1, transform2);
				if (b3IsSeparationAxis(transform, hull2, hull1, output.cache.face)) {
					return;
				}
			}
		}
		else {
			const b3HalfEdge* edge1 = hull1->GetEdge(output.cache.edge1);
			const b3HalfEdge* twin1 = hull1->GetEdge(output.cache.edge1 + 1);

			const b3HalfEdge* edge2 = hull2->GetEdge(output.cache.edge2);
			const b3HalfEdge* twin2 = hull2->GetEdge(output.cache.edge2 + 1);

			b3Transform transform = b3MulT(transform2, transform1);
			b3Vec3 C1 = transform.translation;

			b3Vec3 P1 = transform * hull1->GetVertex(edge1->origin);
			b3Vec3 Q1 = transform * hull1->GetVertex(twin1->origin);
			b3Vec3 E1 = Q1 - P1;

			b3Vec3 U1 = transform.rotation * hull1->GetPlane(edge1->face).normal;
			b3Vec3 V1 = transform.rotation * hull1->GetPlane(twin1->face).normal;

			b3Vec3 P2 = hull2->GetVertex(edge2->origin);
			b3Vec3 Q2 = hull2->GetVertex(twin2->origin);
			b3Vec3 E2 = Q2 - P2;

			b3Vec3 U2 = hull2->GetPlane(edge2->face).normal;
			b3Vec3 V2 = hull2->GetPlane(twin2->face).normal;

			if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2)) {
				r32 distance = b3Project(P1, E1, P2, E2, C1);
				if (distance > 0.0f) {
					return;
				}
			}
		}
	}
	*/

	// Query first hull minimum penetration axis and distance.
	b3FaceQuery faceQuery1;
	b3QueryFaceDirections(faceQuery1, transform1, hull1, transform2, hull2);
	if (faceQuery1.distance > B3_ZERO) {
		output.cache.shape = 1;
		output.cache.face = faceQuery1.index;
		output.cache.type = 1;
		return;
	}

	// Query second hull minimum penetration axis and distance.
	b3FaceQuery faceQuery2;
	b3QueryFaceDirections(faceQuery2, transform2, hull2, transform1, hull1);
	if (faceQuery2.distance > B3_ZERO) {
		output.cache.shape = 2;
		output.cache.face = faceQuery2.index;
		output.cache.type = 1;
		return;
	}

	// Query hulls edges minimum penetration directions.
	b3EdgeQuery edgeQuery;
	b3QueryEdgeDirections(edgeQuery, transform1, hull1, transform2, hull2);
	if (edgeQuery.distance > B3_ZERO) {
		output.cache.type = 2;
		output.cache.edge1 = edgeQuery.index1;
		output.cache.edge2 = edgeQuery.index2;
		return;
	}

	const r32 kRelEdgeTolerance = r32(0.90); //90%
	const r32 kRelFaceTolerance = r32(0.95); //95%
	const r32 kAbsTolerance = r32(0.5) * B3_LINEAR_SLOP;

	// Favor face contacts over edge contacts.
	r32 maxFaceSeparation = b3Max(faceQuery1.distance, faceQuery2.distance);
	if (edgeQuery.distance > kRelEdgeTolerance * maxFaceSeparation + kAbsTolerance) {
		b3CreateEdgeContact(output, edgeQuery, transform1, hull1, transform2, hull2);
	}
	else {
		// Favor first hull face to avoid face flip-flops. 
		if (faceQuery2.distance > kRelFaceTolerance * faceQuery1.distance + kAbsTolerance) {
			// 2 = reference, 1 = incident.
			b3CreateFaceContact(output, faceQuery2, transform2, hull2, transform1, hull1, true);
		}
		else {
			// 1 = reference, 2 = incident.
			b3CreateFaceContact(output, faceQuery1, transform1, hull1, transform2, hull2, false);
		}
	}
#ifdef _DEBUG
	// Make sure all contact IDs are unique.
	for (u32 i = 0; i < output.pointCount; ++i) {
		for (u32 j = i + 1; j < output.pointCount; ++j) {
			b3Assert(output.points[i].id.key != output.points[j].id.key);
		}
	}
#endif
}
