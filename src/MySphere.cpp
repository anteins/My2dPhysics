#include "MySphere.h"
#include "Aabb.h"

using namespace My;

void MySphere::GetAabb(const glm::mat4& trans, glm::vec3& aabbMin, glm::vec3& aabbMax) const
{
	/*TransformAabb(m_fRadius, m_fRadius, trans,
		aabbMin, aabbMax);*/
}

void MySphere::SetSize()
{
	/*double x = 0;
	double y = 0;

	float f = 1.25 - m_fRadius;
	float x = 0; 
	float y = m_fRadius;
	while (x <= y)
	{
		point(x + cx, y + cy);
		point(x + cx, cy - y);
		point(cx - x, cy + y);
		point(cx - x, cy - y);
		point(y + cy, x + cx);
		point(cy - y, x + cx);
		point(cy + y, cx - x);
		point(cy - y, cx - x);

		if (f < 0) 
		{
			f += 0.2*x + 0.03;
		}
		else 
		{ 
			f += 0.2*(x - y) + 0.05; y = y - 0.1; 
		}
		x = x + 0.1;
	}*/
}

void MySphere::UpdateBound()
{
}