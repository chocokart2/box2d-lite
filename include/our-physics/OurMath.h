/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef OUR_MATH_H
#define OUR_MATH_H

#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdlib.h>

const float k_pi = 3.14159265358979323846264f;

// friend 위해
class OurArbiter;
class OurArbiterKey;
class OurContact;
class OurBody;
class OurJoint;
class OurWorld;
class M22;
class MainClass;
class OurCollideClass;

class V2
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurBody;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class M22;
	friend class OurCollideClass;
private:
	float x, y;

public:
	V2() {}
	V2(float x, float y) : x(x), y(y) {}

	void Set(float x_, float y_) { x = x_; y = y_; }

	V2 operator -() { return V2(-x, -y); }

	void operator += (const V2& v)
	{
		x += v.x; y += v.y;
	}
	void operator -= (const V2& v)
	{
		x -= v.x; y -= v.y;
	}
	void operator *= (float a)
	{
		x *= a; y *= a;
	}
	float Length() const
	{
		return sqrtf(x * x + y * y);
	}

	// 이 함수들은 비상용 GetSet 함수입니다.
	float getX() const { return x; }
	float getY() const { return y; }
	
	void setX(float _x) { x = _x; }
	void setY(float _y) { y = _y; }


};

class M22
{
	friend class OurArbiter;
	friend class OurArbiterKey;
	friend class OurContact;
	friend class OurBody;
	friend class OurJoint;
	friend class OurWorld;
	friend class MainClass;
	friend class V2;
	friend class OurCollideClass;
private:
	V2 col1, col2;

public:
	M22() {}
	M22(float angle)
	{
		float c = cosf(angle), s = sinf(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}

	M22(const V2& col1, const V2& col2) : col1(col1), col2(col2) {}

	M22 Transpose() const
	{
		return M22(V2(col1.x, col2.x), V2(col1.y, col2.y));
	}

	M22 Invert() const
	{
		float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		M22 B;
		float det = a * d - b * c;
		assert(det != 0.0f);
		det = 1.0f / det;
		B.col1.x = det * d;	B.col2.x = -det * b;
		B.col1.y = -det * c;	B.col2.y = det * a;
		return B;
	}
	// 이 함수들은 비상용 GetSet 함수입니다.

	V2 GetCol1() const { return col1; }
	V2 GetCol2() const { return col2; }

	void setCol1(V2 value) { col1 = value; }
	void setCol2(V2 value) { col2 = value; }
};

inline float Dot(const V2& a, const V2& b)
{
	return a.getX() * b.getX() + a.getY() * b.getY();
}

inline float Cross(const V2& a, const V2& b)
{
	return a.getX() * b.getY() - a.getY() * b.getX();
}

inline V2 Cross(const V2& a, float s)
{
	return V2(s * a.getY(), -s * a.getX());
}

inline V2 Cross(float s, const V2& a)
{
	return V2(-s * a.getY(), s * a.getX());
}

inline V2 operator * (const M22& A, const V2& v)
{
	return V2(A.GetCol1().getX() * v.getX() + A.GetCol1().getX() * v.getY(), A.GetCol1().getY() * v.getX() + A.GetCol1().getY() * v.getY());
}

inline V2 operator + (const V2& a, const V2& b)
{
	return V2(a.getX() + b.getX(), a.getY() + b.getY());
}

inline V2 operator - (const V2& a, const V2& b)
{
	return V2(a.getX() - b.getX(), a.getY() - b.getY());
}

inline V2 operator * (float s, const V2& v)
{
	return V2(s * v.getX(), s * v.getY());
}

inline M22 operator + (const M22& A, const M22& B)
{
	return M22(A.GetCol1() + B.GetCol1(), A.GetCol2() + B.GetCol1());
}

inline M22 operator * (const M22& A, const M22& B)
{
	return M22(A * B.GetCol1(), A * B.GetCol1());
}

inline float Abs(float a)
{
	return a > 0.0f ? a : -a;
}

inline V2 Abs(const V2& a)
{
	return V2(fabsf(a.getX()), fabsf(a.getY()));
}

inline M22 Abs(const M22& A)
{
	return M22(Abs(A.GetCol1()), Abs(A.GetCol2()));
}

// 만약 죽음의 다이아몬드가 생겨난다면 여기를 조작하여 주석화시키십시오.
///*
inline float Sign(float x)
{
	return x < 0.0f ? -1.0f : 1.0f;
}

inline float Min(float a, float b)
{
	return a < b ? a : b;
}

inline float Max(float a, float b)
{
	return a > b ? a : b;
}

inline float Clamp(float a, float low, float high)
{
	return Max(low, Min(a, high));
}

template<typename T> inline void Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

// Random number in range [-1,1]
inline float Random()
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = 2.0f * r - 1.0f;
	return r;
}

inline float Random(float lo, float hi)
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = (hi - lo) * r + lo;
	return r;
}
/**/
#endif