#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyVector.h"
#include "MyShader.h"
#include "MyMotionState.h"

enum GeometryType
{
	kBox,
	kSphere,
	kCylinder,
	kCone,
	kPlane,
	kCapsule,
	kTriangle
};

typedef struct AabbBound
{
	glm::vec4 min;
	glm::vec4 max;
}AabbBound;

class MyGeometry 
{
public:
	MyGeometry(GeometryType geometry_type):
		m_kGeometryType(geometry_type), 
		m_motionState(nullptr)
	{
		this->color = glm::vec4(1.0f);
	}
	MyGeometry() = delete;
	virtual ~MyGeometry() = default;

	virtual void InitShape(MyMotionState* motionState) = 0;
	virtual void UpdateBound() = 0;
	virtual void Render(MyShader* ourShader, const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection) = 0;

	virtual glm::vec3 GetPoint(unsigned int index) = 0;
	virtual glm::vec3 GetLocalPoint(unsigned int index) = 0;

	void SetColor(glm::vec4 color);

	//3rd
	// GetAabb returns the axis aligned bounding box in the coordinate frame of the given m_motionState trans.
	virtual void GetAabb(const glm::mat4& trans,
		glm::vec3& aabbMin,
		glm::vec3& aabbMax) const = 0;

	virtual void GetBoundingSphere(glm::vec3& center, float& radius) const;

	// GetAngularMotionDisc returns the maximum radius needed for Conservative Advancement to handle 
	// time-of-impact with rotations.
	virtual float GetAngularMotionDisc() const;

	// CalculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	// result is conservative
	void CalculateTemporalAabb(const glm::mat4& curTrans,
		const glm::vec3& linvel,
		const glm::vec3& angvel,
		float timeStep,
		glm::vec3& temporalAabbMin,
		glm::vec3& temporalAabbMax) const;

	GeometryType GetGeometryType() const { return m_kGeometryType; };

protected:
	GeometryType m_kGeometryType;
	float m_fMargin;
	int m_pointCount;

	glm::vec4 color;

	MyMotionState * m_motionState;

	unsigned int VAO;
	unsigned int VBO;
	unsigned int EBO;
};