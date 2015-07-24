#include "b3Polyhedron.h"
#include "b3Hull.h"

b3Polyhedron::b3Polyhedron() {
	m_hull = nullptr;
}

b3Polyhedron::~b3Polyhedron() {
}

void b3Polyhedron::ComputeMass(b3MassData* massData, r32 density) const {
	b3Assert(m_hull);
	b3Assert(m_hull->vertCount >= 3);

	// Simply, we will pick the block local inertia tensor.
	// @todo Assume triangles and integrate to get the approximate tensor.
	// @note Use David Eberly's Game Physics (2006) approach.

	b3AABB aabb;
	aabb.ComputeAabbFromPointArray(m_hull->vertices, m_hull->vertCount);

	r32 w = aabb.Width();
	r32 h = aabb.Height();
	r32 d = aabb.Depth();

	r32 volume = w * h * d;
	massData->mass = volume * density;

	r32 ww = w * w;
	r32 hh = h * h;
	r32 dd = d * d;

	r32 invTwelve = B3_ONE / r32(12.0);
	r32 invTwelveMass = invTwelve * massData->mass;

	// Inertia tensor in local space.
	massData->I.x.x = invTwelveMass * (hh + dd);
	massData->I.y.y = invTwelveMass * (ww + dd);
	massData->I.z.z = invTwelveMass * (ww + hh);

	// Compute average particle positions.
	massData->center.SetZero();
	for ( u32 i = 0; i < m_hull->vertCount; ++i ) {
		massData->center += m_hull->vertices[i];
	}
	massData->center *= B3_ONE / r32(m_hull->vertCount);
}

void b3Polyhedron::ComputeAabb(b3AABB& output, const b3Transform& transform) const {
	b3Assert(m_hull);
	b3Assert(m_hull->vertCount >= 3);
	output.ComputeAabbFromPointArray(m_hull->vertices, m_hull->vertCount, transform);
}

// Christer Ericson, Real-Time Collision Detection (2005), p. 198.
bool b3Polyhedron::RayCast(const b3RayCastInput& input, b3RayCastOutput& output, const b3Transform& transform) const {
	b3Assert(m_hull);
	b3Assert(m_hull->faceCount > 0);

	// Perform computations in the local space of the hull.
	b3Vec3 a = b3MulT(transform.rotation, input.p1 - transform.translation);
	b3Vec3 b = b3MulT(transform.rotation, input.p2 - transform.translation);
	b3Vec3 direction = b - a;
	
	r32 tfirst = B3_ZERO;
	r32 tlast = input.maxFraction;
	i32 index = -1;

	b3Assert(m_hull->faceCount >= 3);

	// Test ray against each plane.
	for (u32 i = 0; i < m_hull->faceCount; i++) {
		const b3Plane& plane = m_hull->planes[i];
		r32 numerator = -b3Distance(plane, a);
		r32 denominator = b3Dot(plane.normal, direction);
		
		if (denominator == B3_ZERO) {
			// The ray is parallel to the plane.
			if (numerator > B3_ZERO) {
				// Then return false if ray segment lies in front (outside) of the plane.
				return false;
			}
		}
		else {
			// Compute parameterized t value for intersection with current plane.
			r32 t = numerator / denominator;
			if (denominator < B3_ZERO) {
				// When entering halfspace, update tfirst if t is larger.
				if (t > tfirst) {
					tfirst = t;
					index = i;
				}
			}
			else {
				// When exiting halfspace, update tlast if t is smaller.
				if (t < tlast) {
					tlast = t;
				}
			}
			// Exit with �no intersection� if intersection becomes empty.
			if (tfirst > tlast) {
				return false;
			}
		}
	}

	if (index != -1) {
		output.fraction = tfirst;
		output.normal = transform.rotation * m_hull->GetPlane(index).normal;
	}

	// If the ray is contained in all planes then it intersects.
	return true;
}
