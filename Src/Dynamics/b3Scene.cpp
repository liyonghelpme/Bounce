#include "b3Scene.h"
#include "b3Body.h"
#include "b3Island.h"
#include "b3SceneListeners.h"
#include "Contacts\b3Contact.h"
#include "..\Collision\Shapes\b3Shape.h"
#include "..\Common\b3Draw.h"

b3Scene::b3Scene() {
	m_contactGraph.m_blockAllocator = &m_blockAllocator;
	m_flags = 0;
	m_bodyList = nullptr;
	m_bodyCount = 0;

	m_flags |= e_clearForcesFlag;

	m_profile.broadPhaseTime = B3_ZERO;
	m_profile.narrowPhaseTime = B3_ZERO;
	m_profile.solverTime = B3_ZERO;
	m_profile.totalTime = B3_ZERO;
}

b3Scene::~b3Scene() {
	// Destroy the shapes that were allocate using b3Alloc.
	b3Body* b = m_bodyList;
	while (b) {
		b->DestroyShapes();
		b = b->m_next;
	}
}

b3Body* b3Scene::CreateBody(const b3BodyDef& def) {
	// Allocate memory for the body.
	void* mem = m_blockAllocator.Allocate(sizeof(b3Body));
	b3Body* b = new(mem) b3Body(def, this);

	// Link the body to this doubly-linked list of rigid bodies.
	b->m_prev = nullptr;
	b->m_next = m_bodyList;
	if (m_bodyList) {
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;

	++m_bodyCount;

	return b;
}

void b3Scene::DestroyBody(b3Body* b) {
	b3Assert(m_bodyCount > 0);

	// Remove the contacts from the body.
	b->DestroyContacts();

	// Remove the shapes from the body.
	b->DestroyShapes();

	// Remove from the world's body list.
	if (b->m_prev) {
		b->m_prev->m_next = b->m_next;
	}
	if (b->m_next) {
		b->m_next->m_prev = b->m_prev;
	}
	if (b == m_bodyList) {
		m_bodyList = b->m_next;
	}

	--m_bodyCount;

	// Free memory.
	b->~b3Body();
	m_blockAllocator.Free(b, sizeof(b3Body));
}

void b3Scene::Solve(const b3TimeStep& step) {
	// Clear all the island flags for the current step.
	for (b3Body* b = m_bodyList; b; b = b->m_next) {
		b->m_flags &= ~b3Body::e_islandFlag;
	}
	for (b3Contact* c = m_contactGraph.m_contactList; c; c = c->m_next) {
		c->m_flags &= ~b3Contact::e_islandFlag;
	}

	b3IslandDef islandDef;
	islandDef.dt = step.dt;
	islandDef.allowSleep = step.sleeping;
	islandDef.allocator = &m_stackAllocator;
	islandDef.bodyCapacity = m_bodyCount;
	islandDef.contactCapacity = m_contactGraph.m_contactCount;
	islandDef.velocityIterations = step.velocityIterations;

	b3Island island(islandDef);

	// Build and simulate awake islands.
	u32 stackSize = m_bodyCount;
	b3Body** stack = (b3Body**)m_stackAllocator.Allocate(stackSize * sizeof(b3Body*));
	for (b3Body* seed = m_bodyList; seed; seed = seed->m_next) {
		// Skip seeds that are on a island.
		if (seed->m_flags & b3Body::e_islandFlag) {
			continue;
		}
		// This seed must be awake.
		if (!seed->IsAwake()) {
			continue;
		}
		// The seed must be dynamic or kinematic.
		if (seed->m_type == e_staticBody) {
			continue;
		}

		// Reset island and stack.
		island.Reset();
		u32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b3Body::e_islandFlag;

		// Do a DFS on the contact graph.
		while (stackCount > 0) {
			// Pop a body from the stack.
			b3Body* b = stack[--stackCount];
			// Add this body to the island.
			island.Add(b);

			// This body must be awake.
			b->SetAwake(true);

			// Don't propagate islands across static bodies to keep the island small.
			if (b->m_type == e_staticBody){
				continue;
			}

			// Get all contacts associated with this body.
			for (b3ContactEdge* ce = b->m_contactList; ce; ce = ce->next) {
				b3Contact* contact = ce->contact;

				// Skip the body if is already on the island.
				if (contact->m_flags & b3Contact::e_islandFlag) {
					continue;
				}

				// Skip contact if the bodies associated with it aren't touching.
				if (!contact->IsTouching()) {
					continue;
				}

				// Skip sensors.
				bool sensorA = contact->m_shapeA->m_isSensor;
				bool sensorB = contact->m_shapeB->m_isSensor;
				if (sensorA || sensorB) {
					continue;
				}

				// Add contact onto the island and mark it.
				island.Add(contact);
				contact->m_flags |= b3Contact::e_islandFlag;

				b3Body* other = ce->other;

				// Skip next propagation if the other body is already on the island.
				if (other->m_flags & b3Body::e_islandFlag) {
					continue;
				}

				// Push the other body onto the stack and mark it.
				b3Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b3Body::e_islandFlag;
			}
		}

		// Integrate velocities, solve velocity constraints, integrate positions.
		island.Solve(m_gravityDir);

		// Cleanup island flags.
		for (u32 i = 0; i < island.bodyCount; ++i) {
			// Allow static bodies to participate in other islands.
			b3Body* b = island.bodies[i];
			if (b->m_type == e_staticBody) {
				b->m_flags &= ~b3Body::e_islandFlag;
			}
		}
	}

	m_stackAllocator.Free(stack);

	{
		b3Time time;

		for (b3Body* b = m_bodyList; b; b = b->m_next) {
			if (b->m_type == e_staticBody) {
				continue;
			}

			// Compute transforms, shape AABBs, and world inertia tensor.
			b->SynchronizeTransform();
			b->SynchronizeShapes();
		}

		m_contactGraph.FindNewContacts();
		time.Update();
		m_profile.broadPhaseTime = time.GetDeltaSecs();
	}
}

void b3Scene::ClearForces() {
	for (b3Body* b = m_bodyList; b; b = b->m_next) {
		b->m_force.SetZero();
		b->m_torque.SetZero();
	}
}

void b3Scene::Step(const b3TimeStep& step) {
	b3Time stepTime;

	if (m_flags & e_bodyAddedFlag) {
		// If new shapes were added we need to find new contacts.
		m_contactGraph.FindNewContacts();
		m_flags &= ~b3Scene::e_bodyAddedFlag;
	}

	{
		b3Time time;
		// Update contact constraints. Destroy ones if they aren't intersecting.
		m_contactGraph.UpdateContacts();
		time.Update();
		m_profile.narrowPhaseTime = time.GetDeltaSecs();
	}
	{
		b3Time time;
		// Solve system's EDOs and MLCPs.
		Solve(step);
		time.Update();
		m_profile.solverTime = time.GetDeltaSecs();
	}
	
	if (m_flags & e_clearForcesFlag) {
		ClearForces();
	}

	stepTime.Update();
	m_profile.totalTime = stepTime.GetDeltaSecs();
}

struct b3QueryCallback {
	bool QueryCallback(i32 proxyID) {
		b3Shape* shape = (b3Shape*)broadPhase->GetUserData(proxyID);
		return listener->ReportShape(shape);
	}
	b3QueryListener* listener;
	const b3BroadPhase* broadPhase;
};

void b3Scene::QueryAABB(b3QueryListener* listener, const b3AABB& aabb) const {
	b3QueryCallback callback;
	callback.broadPhase = &m_contactGraph.m_broadPhase;
	callback.listener = listener;
	m_contactGraph.m_broadPhase.Query(&callback, aabb);
}

struct b3RayCastCallback {
	r32 RayCastCallback(const b3RayCastInput& input, i32 proxyID) {
		b3Shape* shape = (b3Shape*)broadPhase->GetUserData(proxyID);

		b3Transform transform = shape->GetBody()->GetTransform() * shape->GetTransform();

		b3RayCastOutput output;
		bool hit = shape->RayCast(input, output, transform);
		if (hit) {
			r32 fraction = output.fraction;
			b3Vec3 point = (B3_ONE - fraction) * input.p1 + fraction * input.p2;
			b3Vec3 normal = output.normal;

			// Report the intersection and get the new ray cast fraction.
			return listener->ReportShape(shape, point, normal, fraction);
		}

		// Continue from where we stopped.
		return input.maxFraction;
	}

	b3RayCastListener* listener;
	const b3BroadPhase* broadPhase;
};

void b3Scene::RayCast(b3RayCastListener* listener, const b3Vec3& p1, const b3Vec3& p2) const {
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = B3_ONE;// Use B3_MAX_FLOAT for a ray.

	b3RayCastCallback callback;
	callback.listener = listener;
	callback.broadPhase = &m_contactGraph.m_broadPhase;

	m_contactGraph.m_broadPhase.RayCast(&callback, input);
}

void b3Scene::Draw(const b3Draw* draw, u32 flags) const {
	b3Assert(draw);

	if (flags & b3Draw::e_centerOfMassesFlag) {
		for (const b3Body* b = 0; b; b = b->GetNext()) {
			b3Color color;
			color.G = 1.0f;

			const b3Transform& transform = b->GetTransform();

			draw->DrawPoint(transform.translation, color);

			for (const b3Shape* s = b->GetShapeList(); s; s = s->GetNext()) {
				const b3Transform world = transform * s->GetTransform();
				draw->DrawPoint(world.translation, color);
			}
		}
	}

	if (flags & b3Draw::e_broadPhaseFlag) {
		b3Color color1;
		color1.G = 1.0f;
		
		b3Color color2;
		color2.R = 1.0f;

		const b3DynamicAABBTree* tree = &m_contactGraph.m_broadPhase.m_dynamicAabbTree;

		b3Stack<i32> stack;
		stack.Push(tree->m_root);

		while (!stack.Empty()) {
			i32 nodeIndex = stack.Pop();
			
			if (nodeIndex == NULL_NODE) {
				continue;
			}

			const b3DynamicAABBTree::b3Node* node = tree->m_nodes + nodeIndex;

			if (node->IsLeaf()) {
				draw->DrawAABB(node->aabb, color1);
			}
			else {
				draw->DrawAABB(node->aabb, color2);

				stack.Push(node->child1);
				stack.Push(node->child2);
			}
		}
	}

	if (flags & b3Draw::e_contactsFlag) {
		for (const b3Contact* c = m_contactGraph.m_contactList; c; c = c->m_next) {
			const b3Manifold* m = &c->m_manifold;
			
			b3Color color1;
			color1.R = 1.0f;
			color1.G = 1.0f;

			b3Color color2;
			color2.R = 1.0f;

			for (u32 i = 0; i < m->pointCount; ++i) {
				const b3ContactPoint* p = &m->points[i];

				draw->DrawPoint(p->position, color2);
				draw->DrawPoint(p->position + m->normal, color1);
				draw->DrawPoint(p->position + p->tangents[0], color1);
				draw->DrawPoint(p->position + p->tangents[1], color1);
			}
		}

	}
}
