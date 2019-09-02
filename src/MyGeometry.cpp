#include "MyGeometry.h"
#include "MyShader.h"
#include "DebugerManager.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

using namespace My;

void MyGeometry::Draw(MyShader* ourShader, glm::mat4 view, glm::mat4 projection)
{
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(this->vertices), this->vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(this->indices), this->indices, GL_STATIC_DRAW);

	//model = glm::rotate(model, glm::radians(this->rotation), glm::vec3(0.0f, 0.0f, 1.0f));

	//glm::mat4 rotate = this->transform()->GetRotateMatrix();
	//glm::mat4 model = this->transform()->GetTranslateMatrix();

	/*float time = glfwGetTime();
	glm::mat4 trans = glm::mat4(1.0f);
	trans = glm::translate(trans, glm::vec3(0.0f, 1.0f, 0.0f));
	trans = glm::rotate(trans, time, glm::vec3(0.0, 0.0, 1.0));
	trans = glm::scale(trans, glm::vec3(0.4f, 0.4f, 0.4f));
*/
	ourShader->setMat4("model", this->transform()->GetModelMatrix());
	ourShader->setMat4("view", view);
	ourShader->setMat4("projection", projection);
	ourShader->setVec4("vertexColor", this->color);

	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void MyGeometry::SetColor(glm::vec4 color)
{
	this->color = color;
}

void MyGeometry::UpdatePosition(glm::vec3 posDelta)
{
	this->m_motionState->UpdateDelta(posDelta);
	this->UpdateBound();
}

void MyGeometry::UpdateRotate(glm::vec3 angularDelta)
{
	this->m_motionState->UpdateRotate(angularDelta);
}

void MyGeometry::CalculateTemporalAabb(const glm::mat4& curTrans,
	const glm::vec3& linvel,
	const glm::vec3& angvel,
	float timeStep,
	glm::vec3& temporalAabbMin,
	glm::vec3& temporalAabbMax) const
{
	//start with static aabb
	GetAabb(curTrans, temporalAabbMin, temporalAabbMax);

	float temporalAabbMaxx = temporalAabbMax[0];
	float temporalAabbMaxy = temporalAabbMax[1];
	float temporalAabbMaxz = temporalAabbMax[2];
	float temporalAabbMinx = temporalAabbMin[0];
	float temporalAabbMiny = temporalAabbMin[1];
	float temporalAabbMinz = temporalAabbMin[2];

	// add linear motion
	glm::vec3 linMotion = linvel * timeStep;
	///@todo: simd would have a vector max/min operation, instead of per-element access
	if (linMotion[0] > 0.0f)
		temporalAabbMaxx += linMotion[0];
	else
		temporalAabbMinx += linMotion[0];
	if (linMotion[1] > 0.0f)
		temporalAabbMaxy += linMotion[1];
	else
		temporalAabbMiny += linMotion[1];
	if (linMotion[2] > 0.0f)
		temporalAabbMaxz += linMotion[2];
	else
		temporalAabbMinz += linMotion[2];

	//add conservative angular motion
	float angularMotion = angvel.length() * GetAngularMotionDisc() * timeStep;
	glm::vec3 angularMotion3d({ angularMotion, angularMotion, angularMotion });
	temporalAabbMin = glm::vec3({ temporalAabbMinx,temporalAabbMiny,temporalAabbMinz });
	temporalAabbMax = glm::vec3({ temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz });

	temporalAabbMin = temporalAabbMin - angularMotion3d;
	temporalAabbMax = temporalAabbMax + angularMotion3d;
}

//»ñÈ¡°üÎ§Ô²
void MyGeometry::GetBoundingSphere(glm::vec3& center, float& radius) const
{
	glm::mat4 tran = glm::mat4(1.0f);
	//BuildIdentityMatrix(tran);
	glm::vec3 aabbMin, aabbMax;

	GetAabb(tran, aabbMin, aabbMax);

	radius = (aabbMax - aabbMin).length() * 0.5f;
	center = (aabbMin + aabbMax) * 0.5f;
}

float MyGeometry::GetAngularMotionDisc() const
{
	glm::vec3    center;
	float       disc = 0.0f;
	GetBoundingSphere(center, disc);
	disc += center.length();
	return disc;
}