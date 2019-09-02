#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Debugger 
{
public:
	void DrawPoint(glm::vec3 point);

	void DrawLine(glm::vec3 from, glm::vec3 to);
private:

};