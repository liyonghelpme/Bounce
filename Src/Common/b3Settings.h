#ifndef __B3_SETTINGS_H__
#define __B3_SETTINGS_H__

#include <float.h>
#include <assert.h>
#include <new>

typedef signed char	i8;
typedef signed short i16;
typedef signed int i32;
typedef unsigned char u8;
typedef unsigned short u6;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef float r32;
typedef double r64;

// You can modify the following parameters as long
// you're right.

#define B3_ZERO r32(0.0)
#define B3_HALF r32(0.5)
#define B3_ONE r32(1.0)
#define B3_TWO r32(2.0)
#define	B3_EPSILON (FLT_EPSILON)
#define B3_PI r32(3.14159265359)

#define b3Sqrt(x) sqrt(x)
#define b3Abs(x) abs(x)

#define	B3_MAX_FLOAT (FLT_MAX)
#define	B3_MAX_LONG (LONG_MAX)

#define B3_MAX_HULL_VERTICES 100
#define B3_MAX_FACE_VERTICES 10
#define B3_MAX_HULL_FACES 100
#define B3_MAX_HULL_EDGES 2 * B3_MAX_HULL_VERTICES
#define B3_MAX_HULL_HALF_EDGES 2 * B3_MAX_HULL_EDGES

#define B3_WORLD_AABB_EXTENSION r32(0.2)
#define B3_MAX_MANIFOLD_POINTS 8
#define B3_MAX_TANGENT_DIRECTIONS 2

#define B3_SLOP r32(0.005) //5cm
#define B3_BAUMGARTE r32(0.1)
#define B3_GRAVITY_ACC r32(9.8)

#define B3_ONE_SECOND_MICROSECONDS (1000000ULL) //1s=1000000us
#define B3_TIME_TO_SLEEP r32(0.5)

#define B3_SLEEP_LINEAR_TOL r32( 0.05 ) //50cm
#define B3_SLEEP_ANGULAR_TOL r32( (2.0 / 180.0) * B3_PI )

#define b3Assert(c) assert(c)

void* b3Alloc(i32 size);
void b3Free(void* memory);

template <class T>
inline void b3Swap(T& a, T& b) {
	T tmp = a;
	a = b;
	b = tmp;
}

#endif
