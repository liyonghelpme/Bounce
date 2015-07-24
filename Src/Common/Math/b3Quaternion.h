/*
******************************************************************************
   Copyright (c) 2015 Irlan Robson http://www.irlanengine.wordpress.com

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
	 claim that you wrote the original software. If you use this software
	 in a product, an acknowledgment in the product documentation would be
	 appreciated but is not required.
   2. Altered source versions must be plainly marked as such, and must not
	 be misrepresented as being the original software.
   3. This notice may not be removed or altered from any source distribution.
*******************************************************************************
*/

#ifndef __B3_QUATERNION_H__
#define __B3_QUATERNION_H__

#include "b3Mat33.h"

struct b3Quaternion {
	// A quaternion class used to (precisely) represent
	// a rigid body orientation.
	
	// Set to identity.
	b3Quaternion() {
		a = b = c = B3_ZERO;
		d = B3_ONE;
	}

	// Constructor.
	b3Quaternion(r32 _a, r32 _b, r32 _c, r32 _d) : 
		a(_a), b(_b), c(_c), d(_d) {
	}

	// Add a quaternion to this one.
	b3Quaternion& operator+=(const b3Quaternion& q) {
		a += q.a;
		b += q.b;
		c += q.c;
		d += q.d;
		return (*this);
	}

	// Compute this quaternion given an axis and the angle of rotation about
	// the axis.
	void Set(const b3Vec3& axis, r32 radians) {
		r32 halfAngle = B3_HALF * radians;
		r32 s = ::sin(halfAngle);
		a = s * axis.x;
		b = s * axis.y;
		c = s * axis.z;
		d = ::cos(halfAngle);
	}

	// Normalize the quarternion (if needed).
	b3Quaternion& Normalize() {
		r32 s = a * a + b * b + c * c + d * d;
		s = s == B3_ZERO ? B3_ONE : B3_ONE / b3Sqrt(s);
		if (s > B3_EPSILON) {
			a *= s;
			b *= s;
			c *= s;
			d *= s;
		}
		return (*this);
	}

	// Convert this quaternion to a rotation matrix.
	void ToRotationMatrix(b3Mat33& R) const {
		r32 aa = a + a;
		r32 bb = b + b;
		r32 cc = c + c;

		r32 aaa = a * aa;
		r32 abb = a * bb;
		r32 acc = a * cc;

		r32 bbb = b * bb;
		r32 bcc = b * cc;
		
		r32 ccc = c * cc;
		
		r32 daa = d * aa;		
		r32 dbb = d * bb;
		r32 dcc = d * cc;

		R.x.x = B3_ONE - bbb - ccc;
		R.x.y = abb + dcc;
		R.x.z = acc - dbb;
		
		R.y.x = abb - dcc;
		R.y.y = B3_ONE - aaa - ccc;
		R.y.z = bcc + daa;
		
		R.z.x = acc + dbb;
		R.z.y = bcc - daa;
		R.z.z = B3_ONE - aaa - bbb;
	}

	r32 a, b, c, d;
};

// Compute the scalar-quaternion product (r = s * q).
inline b3Quaternion operator*(r32 s, const b3Quaternion& q) {
	return b3Quaternion(s * q.a, s * q.b, s * q.c, s * q.d);
}

// Compute the quaternion-quaternion product (c = a + b).
inline b3Quaternion operator*(const b3Quaternion& a, const b3Quaternion& b) {
	return b3Quaternion(
		a.d * b.a + a.a * b.d + a.b * b.c - a.c * b.b,
		a.d * b.b + a.b * b.d + a.c * b.a - a.a * b.c,
		a.d * b.c + a.c * b.d + a.a * b.b - a.b * b.a,
		a.d * b.d - a.a * b.a - a.b * b.b - a.c * b.c
		);
}

#endif
