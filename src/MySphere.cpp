#include "MySphere.h"
#include "Aabb.h"
#include "GraphicsManager.h"

using namespace My;

void MySphere::GetAabb(const glm::mat4& trans, glm::vec3& aabbMin, glm::vec3& aabbMax) const
{
	/*TransformAabb(m_fRadius, m_fRadius, trans,
		aabbMin, aabbMax);*/
}

void MySphere::InitShape(MyMotionState* motionState)
{
	glm::vec3 centerPosition = glm::vec3(0.0f);
	this->m_motionState = motionState;

	glm::vec3 vers2[412];
	int offset = 12;
	int num_segments = 400;
	glm::vec3 centerPos = glm::vec3(0.0f);

	for (int i = 0; i < num_segments; i++)
	{
		//get the current angle 
		float theta = 2.0f * PI * float(i) / float(num_segments);

		float x = this->m_fRadius * cosf(theta);//calculate the x component 
		float y = this->m_fRadius * sinf(theta);//calculate the y component 

		vers2[offset + i] = glm::vec3(x + centerPos.x, y + centerPos.y, 0.0f);//output vertex 
	}

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vers2), vers2, GL_STATIC_DRAW);
}

void MySphere::Render(MyShader* ourShader, const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection)
{
	glBindVertexArray(VAO);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	ourShader->setMat4("model", model);
	ourShader->setMat4("view", view);
	ourShader->setMat4("projection", projection);
	ourShader->setVec4("vertexColor", this->color);

	glDrawArrays(GL_LINE_LOOP, 12, 400);

	glBindVertexArray(0);
}

void MySphere::UpdateBound()
{
}