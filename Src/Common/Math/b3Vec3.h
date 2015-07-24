#ifndef __B3_VEC3_H__
#define __B3_VEC3_H__

#include "..\b3Settings.h"

struct b3Vec3 {
	b3Vec3() { 
		// Set to identity.
		x = y = z = B3_ZERO;
	}

	b3Vec3(r32 _x, r32 _y, r32 _z) : x(_x), y(_y), z(_z) {
	}

	void Set(r32 _x, r32 _y, r32 _z) {
		x = _x;
		y = _y;
		z = _z;
	}

	// Set to the zero vector.
	void SetZero() {
		x = y = z = B3_ZERO;
	}

	// Assing other vector to this vector.
	b3Vec3& operator=(const b3Vec3& b) {
		x = b.x;
		y = b.y;
		z = b.z;
		return(*this);
	}

	// Add this vector with another vector.
	b3Vec3& operator+=(const b3Vec3& b) {
		x += b.x;
		y += b.y;
		z += b.z;
		return (*this);
	}

	// Subtract this vector from another vector.
	b3Vec3& operator-=(const b3Vec3& b) {
		x -= b.x;
		y -= b.y;
		z -= b.z;
		return (*this);
	}

	// Multiply this vector by a scalar.
	b3Vec3& operator*=(r32 s) {
		x *= s;
		y *= s;
		z *= s;
		return (*this);
	}

	r32 operator[]( u32 i ) const {
		return reinterpret_cast<const r32 *>(this)[i];
	}

	r32& operator[]( u32 i ) {
		return reinterpret_cast<r32 *>(this)[i];
	}

	r32 x, y, z;
};

// Negate a vector.
inline b3Vec3 operator-(const b3Vec3& v) {
	return b3Vec3(-v.x, -v.y, -v.z);
}

// Compute sum of two vectors.
inline b3Vec3 operator+(const b3Vec3& a, const b3Vec3& b) {
	return b3Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

// Compute subtraction of two vectors.
inline b3Vec3 operator-(const b3Vec3& a, const b3Vec3& b) {
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Compute scalar-vector product.
inline b3Vec3 operator*(r32 s, const b3Vec3& v) {
	return b3Vec3(s * v.x, s * v.y, s * v.z);
}

// Compute the product of twi vectors.
inline b3Vec3 operator*(const b3Vec3& a, const b3Vec3& b) {
	return b3Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Compute the length of a vector.
inline r32 b3Len(const b3Vec3& v) {
	return b3Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Compute the squared length of a vector.
inline r32 b3LenSq(const b3Vec3& v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Compute the dot-product of two vectors.
inline r32 b3Dot(const b3Vec3& a, const b3Vec3& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Compute the cross-product of two vectors.
inline b3Vec3 b3Cross(const b3Vec3& a, const b3Vec3& b) {
	return b3Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Computed the normalized vector of a (non-zero!) vector.
inline b3Vec3 b3Normalize(const b3Vec3& v) {
	//@warning the vector must be normalized.
	r32 invLen = B3_ONE / b3Len(v);
	return invLen * v;
}

// Create a basis matrix given a vector.
inline void b3ComputeBasis(const b3Vec3& a, b3Vec3* b, b3Vec3* c) {
	// References: Box2D.
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component of a
	// unit vector must be greater or equal to 0.57735.
	if ( b3Abs(a.x) >= r32(0.57735027) ) {
		b->Set(a.y, -a.x, B3_ZERO);
	}
	else {
		b->Set(B3_ZERO, a.z, -a.y);
	}

	*b = b3Normalize(*b);
	*c = b3Cross(a, *b);
}

#endif