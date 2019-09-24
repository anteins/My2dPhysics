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
	this->m_motionState = motionState;

	float width_half = m_vHalfExtents.x / 2;
	float height_half = m_vHalfExtents.y / 2;

	float vertices[12];
	unsigned int indices[6];

	vertices[0] = centerPosition.x + width_half;
	vertices[1] = centerPosition.y + height_half;
	vertices[2] = centerPosition.z;

	vertices[3] = centerPosition.x + width_half;
	vertices[4] = centerPosition.y - height_half;
	vertices[5] = centerPosition.z;

	vertices[6] = centerPosition.x - width_half;
	vertices[7] = centerPosition.y - height_half;
	vertices[8] = centerPosition.z;

	vertices[9] = centerPosition.x - width_half;
	vertices[10] = centerPosition.y + height_half;
	vertices[11] = centerPosition.z;

	this->m_localMin = glm::vec4(vertices[0], vertices[1], vertices[2], 1.0f);
	this->m_localMax = glm::vec4(vertices[6], vertices[7], vertices[8], 1.0f);

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
	glm::mat4 transition = this->m_motionState->GetMatrix();

	glm::vec4 maxPos = m_localMin;
	glm::vec4 minPos = m_localMax;

	m_aabbBound.min = transition * minPos;
	m_aabbBound.max = transition * maxPos;
}
