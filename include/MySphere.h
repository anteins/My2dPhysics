#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyGeometry.h"

const static float PI = 3.1415926f;
const static double toPI = PI / 180.0;

class MySphere : public MyGeometry 
{
public:
	MySphere() = delete;
	MySphere(glm::vec3 centerPos, float radius) : MyGeometry(GeometryType::kSphere), m_fRadius(radius){};

	void InitShape(glm::vec3 centerPosition, MyMotionState* motionState);
	void UpdateBound();
	void Render(MyShader* ourShader, const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection);

	glm::vec3 GetPoint(unsigned int index) { return glm::vec3(0); }

	void GetAabb(const glm::mat4& trans,
		glm::vec3& aabbMin,
		glm::vec3& aabbMax) const final;

	float GetRadius() const { return m_fRadius; }

protected:
	float m_fRadius;
	//float vertices[421];
};