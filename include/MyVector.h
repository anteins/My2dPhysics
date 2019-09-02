#pragma once
#include <algorithm>

namespace My 
{
	class Vec2
	{
	public:
		Vec2() { this->x = 0; this->y = 0; }

		Vec2(float x, float y) 
		{
			this->x = x;
			this->y = y;
		}

		float x;
		float y;

		/*const float& operator[](size_t idx) const 
		{
			return (*const_cast<Vec2*>(this))[idx];
		}

		Vec2 operator-() const 
		{ 
			return { -x, -y }; 
		}*/

		float magnitude() const 
		{
			return sqrt(x * x + y * y);
		}

		void Normalize() 
		{
			float m = magnitude();
			this->x /= m;
			this->y /= m;
		}

		static inline float Dot(const Vec2& a, const Vec2& b)
		{
			return a.x * b.x + a.y * b.y;
		}

		static inline float Cross(const Vec2& a, const Vec2& b)
		{
			return a.x * b.y - a.y * b.x;
		}

	private:
		
	};

	static inline Vec2 operator+(const Vec2& a, const Vec2& b)
	{
		return { a.x + b.x, a.y + b.y };
	}

	static inline void operator+=(Vec2& a, const Vec2& b)
	{
		a = a + b;
	}

	static inline Vec2 operator-(const Vec2& a, const Vec2& b)
	{
		return { a.x - b.x, a.y - b.y };
	}

	static inline void operator-=(Vec2& a, const Vec2& b)
	{
		a = a - b;
	}

	static inline Vec2 operator*(const Vec2& a, const Vec2& b)
	{
		return { a.x * b.x, a.y * b.y };
	}

	static inline Vec2 operator*(const Vec2& a, float b)
	{
		return { a.x * b, a.y * b };
	}

	static inline void operator*=(Vec2& a, float b)
	{
		a = a * b;
	}

	static inline Vec2 operator/(const Vec2& a, float b)
	{
		return { a.x / b, a.y / b };
	}

	static inline void operator/=(Vec2& a, float b)
	{
		a = a / b;
	}
}
