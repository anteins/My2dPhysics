#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Rigidbody2D.h"

using namespace My;

class ContactData 
{
public:
	glm::vec3 contactNormal;
	glm::vec3 contactPoint;
	int penetration;
	Rigidbody2D* body[2];
	float restitution;
};