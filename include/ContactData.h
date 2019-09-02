#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class ContactData 
{
public:
	glm::vec3 collisionNormal;
	int pentration;
};