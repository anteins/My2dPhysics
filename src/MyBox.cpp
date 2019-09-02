#include "MyBox.h"
#include "Aabb.h"

using namespace My;

void MyBox::GetAabb(const glm::mat4& trans, glm::vec3& aabbMin, glm::vec3& aabbMax) const
{
	TransformAabb(m_vHalfExtents, m_fMargin, trans,
		aabbMin, aabbMax);
}

//	3 ------ 0
//	|    A   |
//  2 ------ 1
void MyBox::SetSize()
{
	glm::vec4 localPosition = this->m_motionState->localPosition();

	float width_half = m_vHalfExtents.x / 2;
	float height_half = m_vHalfExtents.y / 2;

	this->vertices[0] = localPosition.x + width_half;
	this->vertices[1] = localPosition.y + height_half;
	this->vertices[2] = localPosition.z;

	this->vertices[3] = localPosition.x + width_half;
	this->vertices[4] = localPosition.y - height_half;
	this->vertices[5] = localPosition.z;

	this->vertices[6] = localPosition.x - width_half;
	this->vertices[7] = localPosition.y - height_half;
	this->vertices[8] = localPosition.z;

	this->vertices[9] = localPosition.x - width_half;
	this->vertices[10] = localPosition.y + height_half;
	this->vertices[11] = localPosition.z;

	UpdateBound();

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
	glBufferData(GL_ARRAY_BUFFER, sizeof(this->vertices), this->vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(this->indices), this->indices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
}

void MyBox::UpdateBound() 
{
	glm::mat4 transition = this->m_motionState->GetModelMatrix();
	glm::vec3 maxPos = glm::vec3(this->vertices[6], this->vertices[7], this->vertices[8]);
	glm::vec3 minPos = glm::vec3(this->vertices[0], this->vertices[1], this->vertices[2]);

	m_aabbBound.min = transition * glm::vec4(maxPos, 1.0f);
	m_aabbBound.max = transition * glm::vec4(minPos, 1.0f);
}
