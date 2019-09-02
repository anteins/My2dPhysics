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
	glm::vec3 min;
	glm::vec3 max;
}AabbBound;

class MyGeometry 
{
public:
	MyGeometry(GeometryType geometry_type):m_kGeometryType(kBox)
	{
		this->m_motionState = new MyMotionState();
		this->m_motionState->SetLocalPosition(glm::vec4(1.0f));
	}
	MyGeometry() = delete;
	virtual ~MyGeometry() = default;

	virtual void UpdateBound() = 0;
	virtual void SetSize() = 0;

	void UpdatePosition(glm::vec3 posDelta);
	void UpdateRotate(glm::vec3 angularDelta);
	void SetColor(glm::vec4 color);
	void Draw(MyShader* ourShader, glm::mat4 view, glm::mat4 projection);
	AabbBound GetBound() { return m_aabbBound; }
	//void SetShader(unsigned int VAO, unsigned int VBO, unsigned int EBO);

	float GetWidth() { return this->width; }
	float GetHeight() { return this->height; }
	MyMotionState* transform() { return this->m_motionState; }





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

	float width;
	float height;
	glm::vec4 color;
	MyMotionState* m_motionState;
	AabbBound m_aabbBound;

	unsigned int VAO;
	unsigned int VBO;
	unsigned int EBO;

	float angular;

	float vertices[12];
	unsigned int indices[6];
};