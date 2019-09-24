#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyGeometry.h"
#include "MyMotionState.h"

class MyBox : public MyGeometry 
{
public:
	MyBox() = delete;
	MyBox(glm::vec3 centerPosition,
		glm::vec3 halfExtents):
		MyGeometry(GeometryType::kBox), 
		m_vHalfExtents(halfExtents),
		m_direction(glm::vec4(0.0, 1.0, 0.0, 0.0)){};

	//override
	void InitShape(glm::vec3 centerPosition, MyMotionState* motionState);
	void UpdateBound();
	void Render(MyShader* ourShader, const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection);

	AabbBound GetBound() { return m_aabbBound; }
	float GetWidth() { return this->width; }
	float GetHeight() { return this->height; }
	glm::vec3 GetDirection() { return this->m_motionState->GetMatrix() *  this->m_direction;}

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

	float width;
	float height;

	glm::vec4 m_direction;

	AabbBound m_aabbBound;
	glm::vec4 m_localMin;
	glm::vec4 m_localMax;
};