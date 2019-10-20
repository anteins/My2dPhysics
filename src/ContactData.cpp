#include "ContactData.h"
#include "DebugerManager.h"

using namespace My;

void Contact::CalculateInternals(float dt) 
{
	if (!this->body[0])
		return;

	Rigidbody2D* body1 = this->body[0];
	Rigidbody2D* body2 = this->body[1];

	//create contact basic
	CreateContactBasic();

	//碰撞点的相对位置
	this->relativePosition[0] = this->contactPoint - this->body[0]->GetPosition();
	if(this->body[1])
		this->relativePosition[1] = this->contactPoint - this->body[1]->GetPosition();

	//碰撞点的相对速度
	this->contactVelocity = this->CalculateLocalVelocity(0, dt);
	if (this->body[1]) 
		this->contactVelocity -= this->CalculateLocalVelocity(1, dt);
}

void Contact::CreateContactBasic() 
{
	glm::vec3 otherAxis[2];

	if (abs(this->contactNormal.x) > abs(this->contactNormal.y)) 
	{
		//z-Axis
		otherAxis[0] = glm::vec3(0, 0, 1);
		//y-Axis
		otherAxis[1] = glm::cross(this->contactNormal, otherAxis[0]);
	}
	else 
	{
		//z-Axis
		otherAxis[0] = glm::vec3(0, 0, 1);
		//y-Axis
		otherAxis[1] = glm::cross(this->contactNormal, otherAxis[0]);
	}

	//set component
	this->contactToWorld = glm::mat3(
		this->contactNormal.x, otherAxis[1].x, otherAxis[0].x,
		this->contactNormal.y, otherAxis[1].y, otherAxis[0].y,
		this->contactNormal.z, otherAxis[1].z, otherAxis[0].z
	);

	this->worldToContact = glm::transpose(this->contactToWorld);
}

glm::vec3 Contact::CalculateLocalVelocity(unsigned int index, float dt)
{
	glm::vec3 contactVelocity;
	DebugerManager::PrintVec3(this->contactPoint, "contactPoint: ");
	DebugerManager::PrintVec3(this->body[0]->GetPosition(), "pos_0: ");
	//DebugerManager::PrintVec3(this->body[index]->GetRotate(), "local angluarVel: ");
	//DebugerManager::PrintVec3(this->body[index]->GetVelocity(), "local Vel: ");
	contactVelocity = glm::cross(this->body[index]->GetRotate(), this->relativePosition[index]);
	DebugerManager::PrintVec3(contactVelocity, "contactVelocity: ");
	contactVelocity += this->body[index]->GetVelocity();
	contactVelocity = this->worldToContact * contactVelocity;

	glm::vec3 accVelocity = this->body[index]->acceleration * dt;
	accVelocity = this->worldToContact * accVelocity;
	accVelocity.x = 0;
	contactVelocity += accVelocity;

	return contactVelocity;
}

void Contact::ApplyPosition(float dt)
{
	//相交
	glm::vec3 tmpPos = this->body[0]->GetPosition();
	glm::vec3 move = this->penetration * this->contactNormal;
	this->body[0]->SetPosition(move);
}

void Contact::ApplyVelocity(float dt) 
{
	//计算期望速度变化
	float accCausedVelocity = glm::dot(this->body[0]->acceleration, this->contactNormal) * dt;
	if (this->body[1])
	{
		accCausedVelocity -= glm::dot(this->body[1]->acceleration, this->contactNormal) * dt;
	}
	float accCausedSepVelocity = accCausedVelocity;

	// If the velocity is very slow, limit the restitution
	real thisRestitution = restitution;
	if (real_abs(contactVelocity.x) < 0.25f)
	{
		thisRestitution = (real)0.0f;
	}

	this->desiredDeltaVelocity = -this->contactVelocity.x - thisRestitution * ((double)this->contactVelocity.x - accCausedSepVelocity);

	//冲击转矩量 = q_rel * g（g为冲量）,计算单位冲量时，g = contactNormal
	glm::vec3 torquePerUnitImpulse = glm::cross(relativePosition[0], this->contactNormal);
	//角速度变化 = 冲击转矩量 / I
	glm::vec3 deltaRotation = this->body[0]->inverseInertiaTensorWorld * torquePerUnitImpulse;
	//角分量 = 角速度变化 * q_rel
	glm::vec3 deltaVelWorld = glm::cross(deltaRotation, relativePosition[0]);
	float deltaVelocity = glm::dot(deltaVelWorld, this->contactNormal);

	//碰撞点速度 = 角分量 + 线性分量(碰撞坐标系)
	deltaVelocity += this->body[0]->inv_mass;

	if (this->body[1])
	{
		glm::vec3 torquePerUnitImpulse2 = glm::cross(relativePosition[1], this->contactNormal);
		glm::vec3 deltaRotation2 = this->body[1]->inverseInertiaTensorWorld * torquePerUnitImpulse2;
		glm::vec3 rotationVelocity2 = glm::cross(deltaRotation2, relativePosition[1]);
		deltaVelocity += glm::dot(rotationVelocity2, this->contactNormal);

		//线性分量
		deltaVelocity += this->body[1]->inv_mass;
	}

	glm::vec3 impulseContact = glm::vec3(this->desiredDeltaVelocity / deltaVelocity, 0, 0);
	glm::vec3 impulse = this->contactToWorld * impulseContact;
	glm::vec3 impulsiveTorque = glm::cross(relativePosition[0], impulse);

	this->body[0]->velocity += this->body[0]->inv_mass * impulse;
	this->body[0]->rotation += this->body[0]->inverseInertiaTensorWorld * impulsiveTorque;
	
	//printf("+ %f, %f\n", this->desiredDeltaVelocity, deltaVelocity);
	DebugerManager::PrintVec3(this->body[0]->rotation, "rotation: ");
}