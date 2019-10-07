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

	//desiredDeltaVelocity

	void CalculateInternals(float dt);
	void CreateContactBasic();
};