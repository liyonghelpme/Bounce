#ifndef __B3_DRAW_H__
#define __B3_DRAW_H__

#include "Math\b3Math.h"
#include "..\Collision\b3Aabb.h"

struct b3Color {
	b3Color() :
		R(B3_ZERO), 
		G(B3_ZERO), 
		B(B3_ZERO) {
	}
	r32 R, G, B;
};

class b3Draw {
public :
	// Implement this interface and call it in b3Scene::Draw(...) to debug the physics scene.
	// Use these bit flags to tell the scene what needs to be rendered.
	enum b3DrawFlags {
		e_centerOfMassesFlag = 0x0001,
		e_contactsFlag = 0x0002,
		e_broadPhaseFlag = 0x0004
	};

	virtual void DrawPoint(const b3Vec3& position, const b3Color& color) const = 0;
	virtual void DrawLine(const b3Vec3& a, const b3Vec3& b, const b3Color& color) const = 0;
	virtual void DrawAABB(const b3AABB& aabb, const b3Color& color) const = 0;
};

#endif