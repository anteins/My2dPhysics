#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_access.hpp>

class MyMotionState 
{
public:
	MyMotionState():
		m_translateMat(glm::mat4(1.0f)),
		m_rotateMat(glm::mat4(1.0f)),
		m_localPosition(glm::vec4(0.0f)),
		m_zAxis(glm::vec3(0.0, 0.0, 1.0f))
	{}
	
	glm::vec3 GetAxis(unsigned int index)
	{ 
		return this->m_localPosition + glm::column(this->m_translateMat, index);
	}
	
	//Æ½ÒÆ
	void Translate(glm::vec3 posDelta)
	{
		this->m_translateMat = glm::translate(this->m_translateMat, posDelta);
		this->CalcDrividData();
	}

	//Ðý×ª
	void Rotate(float angular)
	{
		this->m_rotateMat = glm::rotate(glm::mat4(1), glm::radians(angular), this->m_zAxis);
		this->CalcDrividData();
	}

	void CalcDrividData() { this->m_transform = this->m_translateMat * this->m_rotateMat; }

	glm::mat4& GetMat44() { return this->m_transform; }

	glm::vec3 Transform(const glm::vec3& vector)  const 
	{
		return this->m_transform * glm::vec4(vector, 1.0);
	}

	glm::vec3 TransformInverse(const glm::vec3& vector) const
	{
		glm::vec3 tmp = vector;
		tmp.x -= this->m_transform[0][3];
		tmp.y -= this->m_transform[1][3];
		tmp.z -= this->m_transform[2][3];

		return glm::vec3(
			tmp.x * this->m_transform[0][0] +
			tmp.y * this->m_transform[1][0] +
			tmp.z * this->m_transform[2][0],

			tmp.x * this->m_transform[0][1] +
			tmp.y * this->m_transform[1][1] +
			tmp.z * this->m_transform[2][1],

			tmp.x * this->m_transform[0][2] +
			tmp.y * this->m_transform[1][2] +
			tmp.z * this->m_transform[2][2]
		);
	}

private:
	glm::mat4 m_transform;
	glm::mat4 m_translateMat;
	glm::mat4 m_rotateMat;
	glm::vec4 m_localPosition;
	glm::vec3 m_zAxis;
};