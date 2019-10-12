#include "MyBox.h"
#include "Aabb.h"
#include "GraphicsManager.h"

using namespace My;

void MyBox::GetAabb(const glm::mat4& trans, glm::vec3& aabbMin, glm::vec3& aabbMax) const
{
	TransformAabb(m_vHalfExtents, m_fMargin, trans,
		aabbMin, aabbMax);
}

//	3 ------ 0
//	|    A   |
//  2 ------ 1
void MyBox::InitShape(glm::vec3 centerPosition, MyMotionState* motionState)
{
	this->m_pointCount = 4;

	this->m_motionState = motionState;

	float width_half = m_vHalfExtents.x / 2;
	float height_half = m_vHalfExtents.y / 2;

	glm::vec3 vertices[4];
	unsigned int indices[6];

	//右上
	vertices[0] = glm::vec3(
			centerPosition.x + width_half,
			centerPosition.y + height_half,
			centerPosition.z);

	//右下
	vertices[1] = glm::vec3(
		centerPosition.x + width_half,
		centerPosition.y - height_half,
		centerPosition.z);

	//左下
	vertices[2] = glm::vec3(
		centerPosition.x - width_half,
		centerPosition.y - height_half,
		centerPosition.z);

	//左上
	vertices[3] = glm::vec3(
		centerPosition.x - width_half,
		centerPosition.y + height_half,
		centerPosition.z);

	this->m_localMin = glm::vec4(vertices[0], 1.0f);
	this->m_localMax = glm::vec4(vertices[2], 1.0f);

	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 3;
	indices[3] = 1;
	indices[4] = 2;
	indices[5] = 3;

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
}

void MyBox::Render(MyShader* ourShader, const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection)
{
	glBindVertexArray(VAO);

	ourShader->setMat4("model", model);
	ourShader->setMat4("view", view);
	ourShader->setMat4("projection", projection);
	ourShader->setVec4("vertexColor", this->color);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void MyBox::UpdateBound()
{
	glm::mat4 transition = this->m_motionState->GetMat44();

	glm::vec4 maxPos = m_localMin;
	glm::vec4 minPos = m_localMax;

	m_aabbBound.min = transition * minPos;
	m_aabbBound.max = transition * maxPos;
}

glm::vec3 MyBox::GetPoint(unsigned int index) 
{
	////右上
	//vertices[0] = glm::vec3(

	////右下
	//vertices[1] = glm::vec3(

	////左下
	//vertices[2] = glm::vec3(

	////左上
	//vertices[3] = glm::vec3(

	float width_half = m_vHalfExtents.x / 2;
	float height_half = m_vHalfExtents.y / 2;

	if (this->m_pointCount == 4) 
	{
		switch (index)
		{
		default:
		case 0: return glm::vec3(
			width_half,
			height_half,
			0);
		case 1: return glm::vec3(
			width_half,
			-height_half,
			0);
		case 2: return glm::vec3(
			-width_half,
			-height_half,
			0);
		case 3: return glm::vec3(
			-width_half,
			height_half,
			0);
		}
	}
}
