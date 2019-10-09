#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Rigidbody2D.h"

using namespace My;

class ContactData 
{
public:
	Rigidbody2D* body[2];
	glm::vec3 contactNormal;
	glm::vec3 contactPoint;

	float penetration;
	float restitution;

	float desiredDeltaVelocity;

	glm::mat3 contactToWorld;
	glm::mat3 worldToContact;

	glm::vec3 relativePosition[2];
	glm::vec3 contactVelocity;

	//desiredDeltaVelocity

	void CalculateInternals(float dt);
	void CreateContactBasic();
	glm::vec3 CalculateLocalVelocity(unsigned int index, float dt);

	void ApplyPosition(float dt);

	void ApplyVelocity(float dt);
};