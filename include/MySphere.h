#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyGeometry.h"

class MySphere : public MyGeometry 
{
public:
	MySphere() = delete;
	MySphere(glm::vec3 center, float radius) : MyGeometry(GeometryType::kSphere), m_fRadius(radius)
	{
		this->m_motionState->SetLocalPosition(glm::vec4(center, 1.0f));
	};

	void SetSize();
	void UpdateBound();
	void GetAabb(const glm::mat4& trans,
		glm::vec3& aabbMin,
		glm::vec3& aabbMax) const final;

	float GetRadius() const { return m_fRadius; }

protected:
	float m_fRadius;
};