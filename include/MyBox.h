#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyGeometry.h"


class MyBox : public MyGeometry 
{
public:
	MyBox() = delete;
	MyBox(glm::vec3 centerPosition, glm::vec3 halfExtents): 
		MyGeometry(GeometryType::kBox), 
		m_vHalfExtents(halfExtents)
	{
		this->m_motionState->SetLocalPosition(glm::vec4(centerPosition, 1.0f));
		SetSize();
	};

	void UpdateBound();
	void SetSize();

	//3rd
	void GetAabb(const glm::mat4& trans,
		glm::vec3& aabbMin,
		glm::vec3& aabbMax) const final;

	glm::vec3 GetDimension() const { return m_vHalfExtents * 2.0f; }
	glm::vec3 GetDimensionWithMargin() const { return m_vHalfExtents * 2.0f + m_fMargin; }
	glm::vec3 GetHalfExtents() const { return m_vHalfExtents; }
	glm::vec3 GetHalfExtentsWithMargin() const { return m_vHalfExtents + m_fMargin; }

protected:
	glm::vec3 m_vHalfExtents;
};