#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_access.hpp>

class MyMotionState 
{
public:
	MyMotionState():
		m_translate(glm::mat4(1.0f)),
		m_rotate(glm::mat4(1.0f)),
		m_localPosition(glm::vec4(1.0f))
	{}
	
	glm::vec4 position() { return m_localPosition + glm::column(m_translate, 3); }
	glm::vec4 localPosition() {return m_localPosition;}



	void SetLocalPosition(glm::vec4 position) { m_localPosition = position; }
	
	void UpdateDelta(glm::vec3 posDelta)
	{
		this->m_translate = glm::translate(this->m_translate, posDelta);
	}

	void UpdateRotate(glm::vec3 angularDelta)
	{
		this->m_rotate = glm::rotate(this->m_rotate, glm::radians(angularDelta.z), glm::vec3(0.0, 0.0, 1.0f));
	}

	glm::mat4 GetTranslateMatrix() { return this->m_translate; }
	glm::mat4 GetRotateMatrix() { return this->m_rotate; }
	glm::mat4 GetModelMatrix() { return this->m_translate * this->m_rotate; }

	//glm::vec3 GetRotate();
	//glm::vec3 SetRotate(glm::vec3 position);
	//glm::vec3 GetScale();
private:
	glm::mat4 m_translate;
	glm::mat4 m_rotate;

	glm::vec4 m_localPosition;
};