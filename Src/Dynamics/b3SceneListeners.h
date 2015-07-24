#ifndef __B3_SCENE_LISTENERS_H__
#define __B3_SCENE_LISTENERS_H__

#include "..\Common\Math\b3Math.h"

class b3Shape;
class b3Contact;

class b3QueryListener {
public :
	virtual ~b3QueryListener() {}
	virtual bool ReportShape(b3Shape* shape) = 0;
};

class b3RayCastListener {
public:	
	// The user must return the new ray cast fraction.
	// If fraction == zero then the ray cast query will be canceled.
	virtual ~b3RayCastListener() { }
	virtual r32 ReportShape(b3Shape* shape, const b3Vec3& point, const b3Vec3& normal, r32 fraction) = 0;
};

class b3ContactListener {
public:
	// Inherit from this class and set it in the scene to listen for collision events.	
	// Call the functions below to inspect when a shape start/end colliding with another shape.
	/// @warning You cannot create/destroy Bounc3 objects inside these callbacks.
	virtual void BeginContact(b3Contact* contact) = 0;
	virtual void EndContact(b3Contact* contact) = 0;
	virtual void Persisting(b3Contact* contact) = 0;
};

#endif