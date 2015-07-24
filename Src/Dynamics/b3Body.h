#ifndef __B3_BODY_H__
#define __B3_BODY_H__

#include "../Common/b3Settings.h"
#include "../Common/Math/b3Math.h"

class b3Scene;
class b3Shape;
struct b3ShapeDef;
struct b3ContactEdge;

// Static bodies have zero mass and velocity, and therefore they can't move.
// Dynamic bodies have non-zero mass and can move due to internal and external forces.
// Kinematic bodies aren't moved due to external and internal forces but can be moved by the contact solver.
enum b3BodyType {
	e_staticBody = 1,
	e_kinematicBody,
	e_dynamicBody
};

// Pass this definition to the scene to create a new rigid body.
struct b3BodyDef {
	b3BodyDef() {
		type = e_staticBody;
		awake = true;
		active = true;
		userData = nullptr;
		gravityScale = B3_ONE;
	}

	b3BodyType type;
	b3Vec3 position;
	b3Quaternion orientation;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
	r32 gravityScale;
	bool awake;
	bool active;
	void* userData;
};

class b3Body {
public :
	b3Body(const b3BodyDef& def, b3Scene* scene);
	// @note
	// The scene manages the body destruction.

	// Create a new shape for the body given the shape definition and return a pointer to it.
	// The shape passed to the definition it will be cloned and is not recommended modifying 
	// it inside simulation callbacks. 
	// Therefore you can create shapes on the stack memory.
	b3Shape* CreateShape(const b3ShapeDef& def);

	// Remove a shape from the body and deallocated it from the memory.
	void DestroyShape(b3Shape* shape);
	
	// Remove all shapes associated with the body.
	// Remove this if you want to recycle the body, but (physically) doesn't make
	// any sense a bodies without a shape.
	void DestroyShapes();

	// Remove all contacts associated with the body.
	void DestroyContacts();

	// Get the shapes associated with the body.
	const b3Shape* GetShapeList() const;

	// Get the total shapes associated with the body.
	const u32 GetShapeCount() const;

	// Get the type of the body.
	u32 GetType() const;

	// Set the type of the body. 
	// This will reset the current body inertial properties.
	void SetType(b3BodyType type);

	// Get the scene the body belongs to.
	const b3Scene* GetScene() const;
	b3Scene* GetScene();

	// Get the next body on the scene body list.
	const b3Body* GetNext() const;
	b3Body* GetNext();

	// @warning 
	// Manipulating a body transform during the simulation may cause non-physical behaviour.
	const b3Transform& GetTransform() const;
	void SetTransform(const b3Vec3& position, const b3Vec3& axis, r32 radians);

	// Get the user data associated with the body.
	// The user data is usually a game entity.
	void* GetUserData() const;
	
	// Set the user data to the body.
	void SetUserData(void* _userData);

	// Set the awake status of the body.
	void SetAwake(bool flag);
	
	// See if the body is awake.
	bool IsAwake() const;	
	
	// Get the gravity scale of the body. One is used by default.
	r32 GetGravityScale() const;
	
	// Set the gravity scale of the body.
	void SetGravityScale(r32 scale);

	// Apply a force to a specific point (particle) of the body. 
	// If the point isn't the center of mass then a non-zero torque is applied.
	// The body must be dynamic.
	void ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake);
	
	// Apply a force to the body center of mass. Generally this is a external force. 
	// The body must be dynamic.
	void ApplyForceToCenter(const b3Vec3& force, bool wake);
	
	// Apply a torque (angular momentum change) to the body.
	// The body must be dynamic.
	void ApplyTorque(const b3Vec3& torque, bool wake);

	// Apply a linear impulse (linear velocity change) to a specific point (particle) of the body. 
	// If the point isn't the center of mass then a non-zero angular impulse is applied.
	// The body must be dynamic.
	void ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& point, bool wake);
	
	// Apply a angular impulse (angular velocity change) to the body.
	// The body must be dynamic.
	void ApplyAngularImpulse(const b3Vec3& impulse, bool wake);

	// Set the linear velocity of the body. If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetLinearVelocity(const b3Vec3& linearVelocity);
	
	// Set the angular velocity of the body. If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetAngularVelocity(const b3Vec3& angularVelocity);
protected :
	friend class b3Scene;
	friend class b3ContactGraph;
	friend class b3Contact;
	friend class b3Joint;
	friend class b3Island;
	friend class b3ContactSolver;

	enum b3BodyFlags {
		e_awakeFlag = 0x0001,
		e_islandFlag = 0x0002
	};
	
	// Recalculate the mass of the body based on the shapes associated
	// with the body.
	void ResetMassData();

	// Compute the body world inertial tensor and transform.
	void SynchronizeTransform();

	// Compute the shapes world AABBs, and the world inertia tensor.
	// Usually this is called after the body transform is synchronized.
	void SynchronizeShapes();

	// Destroy all contacts associated with the given shape.
	void DestroyContacts(const b3Shape* shape);

	u32 m_type;
	u32 m_flags;
	i32 m_islandID;
	r32 m_sleepTime;

	// To the linked list of bodies.
	b3Scene* m_scene;
	b3Body* m_prev;
	b3Body* m_next;

	// The body holds a list of neighbour contacts.
	b3ContactEdge* m_contactList;

	// The shapes attached to this body.
	b3Shape* m_shapeList;
	u32 m_shapeCount;

	// Application specific data (usually a entity).
	void* m_userData;

	// Rigid body data.
	r32 m_mass, m_invMass;
	b3Mat33 m_invLocalInertia, m_invWorldInertia;
	
	r32 m_gravityScale;

	// The applied forces.
	b3Vec3 m_force;
	b3Vec3 m_torque;

	b3Vec3 m_linearVelocity;
	b3Vec3 m_angularVelocity;
	b3Vec3 m_localCenter;
	b3Vec3 m_worldCenter;
	b3Quaternion m_orientation;
	
	// The body transform.
	b3Transform m_transform;
};

inline const b3Scene* b3Body::GetScene() const {
	return m_scene;
}

inline b3Scene* b3Body::GetScene() {
	return m_scene;
}

inline const b3Body* b3Body::GetNext() const {
	return m_next;
}

inline b3Body* b3Body::GetNext() {
	return m_next;
}

inline u32 b3Body::GetType() const { return m_type; }

inline void b3Body::SetType(b3BodyType type) {
	if (m_type == type) {
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == e_staticBody) {
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();
		SynchronizeShapes();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque.SetZero();

	DestroyContacts();
}

inline void* b3Body::GetUserData() const { return m_userData; }

inline void b3Body::SetUserData(void* userData) { m_userData = userData; }

inline const b3Shape* b3Body::GetShapeList() const { return m_shapeList; }

inline void b3Body::SetTransform(const b3Vec3& position, const b3Vec3& axis, r32 radians) {
	m_worldCenter = position;
	m_orientation.Set(axis, radians);

	SynchronizeTransform();
	SynchronizeShapes();
}

inline const b3Transform& b3Body::GetTransform() const { return m_transform; }

inline void b3Body::SetAwake(bool flag) {
	if (flag) {
		if ( !IsAwake() ) {
			m_flags |= e_awakeFlag;
			m_sleepTime = B3_ZERO;
		}
	}
	else {
		m_flags &= ~e_awakeFlag;
		m_sleepTime = B3_ZERO;
		m_force.SetZero();
		m_torque.SetZero();
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();		
	}
}

inline bool b3Body::IsAwake() const {
	return (m_flags & e_awakeFlag) != 0;
}

inline r32 b3Body::GetGravityScale() const { return m_gravityScale; }

inline void b3Body::SetGravityScale(r32 scale) {
	if (m_type != e_staticBody) {
		m_gravityScale = scale;
	}
}

inline void b3Body::SetLinearVelocity(const b3Vec3& linearVelocity) {
	if (m_type == e_staticBody) {
		return;
	}
	
	if (b3LenSq(linearVelocity) > B3_ZERO) {
		SetAwake(true);
	}

	m_linearVelocity = linearVelocity;
}

inline void b3Body::SetAngularVelocity(const b3Vec3& angularVelocity) {
	if (m_type == e_staticBody) {
		return;
	}

	if (b3LenSq(angularVelocity) > B3_ZERO) {
		SetAwake(true);
	}

	m_angularVelocity = angularVelocity;
}

inline void b3Body::ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake) {
	if (m_type != e_dynamicBody) {
		return;
	}

	if (wake && !IsAwake()) {
		SetAwake(true);
	}

	if (IsAwake()) {
		m_force += force;
		m_torque += b3Cross(point - m_worldCenter, force);
	}
}

inline void b3Body::ApplyForceToCenter(const b3Vec3& force, bool wake) {
	if (m_type != e_dynamicBody) {
		return;
	}

	if (wake && !IsAwake()) {
		SetAwake(true);
	}

	if (IsAwake()) {
		m_force += force;
	}
}

inline void b3Body::ApplyTorque(const b3Vec3& torque, bool wake) {
	if (m_type != e_dynamicBody) {
		return;
	}

	if (wake && !IsAwake()) {
		SetAwake(true);
	}

	if (IsAwake()) {
		m_torque += torque;
	}
}

inline void b3Body::ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& point, bool wake) {
	if (m_type != e_dynamicBody) {
		return;
	}

	if (wake && !IsAwake()) {
		SetAwake(true);
	}

	if (IsAwake()) {
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += m_invWorldInertia * b3Cross(point - m_worldCenter, impulse);
	}
}

inline void b3Body::ApplyAngularImpulse(const b3Vec3& impulse, bool wake) {
	if (m_type != e_dynamicBody) {
		return;
	}

	if (wake && !IsAwake()) {
		SetAwake(true);
	}

	if (IsAwake()) {
		m_angularVelocity += m_invWorldInertia * impulse;
	}
}

#endif