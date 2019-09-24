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
	
	glm::vec4 GetLocalPos() { return m_localPosition; }
	glm::vec4 GetWorldPos() { return m_localPosition + glm::column(m_translateMat, 3); }
	
	//Æ½ÒÆ
	void Translate(glm::vec3 posDelta)
	{
		this->m_translateMat = glm::translate(this->m_translateMat, posDelta);
	}

	//Ðý×ª
	void Rotate(float angular)
	{
		this->m_rotateMat = glm::rotate(this->m_rotateMat, glm::radians(angular), this->m_zAxis);
	}

	glm::mat4 GetTranslateMatrix() { return this->m_translateMat; }
	glm::mat4 GetRotateMatrix() { return this->m_rotateMat; }
	glm::mat4 GetMatrix() { return this->m_translateMat * this->m_rotateMat; }

private:
	glm::mat4 m_translateMat;
	glm::mat4 m_rotateMat;
	glm::vec4 m_localPosition;
	glm::vec3 m_zAxis;
};