#include "ContactData.h"
#include "DebugerManager.h"

using namespace My;

void Contact::SetBodyList(Rigidbody2D* body_a, Rigidbody2D* body_b) 
{
	this->body[0] = body_a;
	if (body_a->IsKinematic()) 
	{
		this->body[0] = nullptr;
	}
	
	this->body[1] = body_b;
	if (body_b->IsKinematic())
	{
		this->body[1] = nullptr;
	}

	if (this->body[0] == nullptr) 
	{
		//swapBodies
		this->contactNormal *= -1;
		Rigidbody2D* temp = body[0];
		body[0] = body[1];
		body[1] = temp;
	}
}

void Contact::CalculateInternals(float dt) 
{
	if (!this->body[0])
		return;

	Rigidbody2D* body1 = this->body[0];
	Rigidbody2D* body2 = this->body[1];

	CreateContactBasic();

	//相对位置
	this->relativePosition[0] = this->contactPoint - this->body[0]->GetPosition();
	if(this->body[1])
		this->relativePosition[1] = this->contactPoint - this->body[1]->GetPosition();

	//相对速度
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
	glm::vec3 contactVelocity(0);
	contactVelocity += glm::cross(this->body[index]->GetRotate(), this->relativePosition[index]);

	//DebugerManager::DrawVector3(this->body[index]->GetPosition(), this->body[index]->GetPosition() + this->relativePosition[index]);
	//DebugerManager::PrintVec3(this->body[index]->GetRotate(), "Rotate Velocity: ");
	//DebugerManager::PrintVec3(this->body[index]->GetVelocity(), "Linear Velocity: ");

	contactVelocity += this->body[index]->GetVelocity();
	contactVelocity = this->worldToContact * contactVelocity;

	glm::vec3 accVelocity = this->body[index]->acceleration * dt;
	accVelocity = this->worldToContact * accVelocity;
	accVelocity.x = 0;
	contactVelocity += accVelocity;

	return contactVelocity;
}

//相交行为处理
void Contact::ApplyPosition(float dt)
{
	glm::vec3 tmpPos = this->body[0]->GetPosition();
	glm::vec3 move = this->penetration * this->contactNormal;
	this->body[0]->SetPosition(move);
}

glm::vec3 Contact::CalculateFrictionlessImpulse(float dt) {
	//冲击转矩量 = q_rel * 冲量
	glm::vec3 torquePerUnitImpulse = glm::cross(relativePosition[0], this->contactNormal);
	//角速度变化 = 冲击转矩量 / 转动惯量
	glm::vec3 deltaRotation = this->body[0]->inverseInertiaTensorWorld * torquePerUnitImpulse;
	//角速度 = 角速度变化 * q_rel
	glm::vec3 deltaVelWorld = glm::cross(deltaRotation, relativePosition[0]);
	//仅碰撞法线方向的分量
	float deltaVelocity = glm::dot(deltaVelWorld, this->contactNormal);
	//点速度 = 角速度 + 线性速度
	deltaVelocity += this->body[0]->inv_mass;

	if (this->body[1])
	{
		glm::vec3 torquePerUnitImpulse2 = glm::cross(relativePosition[1], this->contactNormal);
		glm::vec3 deltaRotation2 = this->body[1]->inverseInertiaTensorWorld * torquePerUnitImpulse2;
		glm::vec3 rotationVelocity2 = glm::cross(deltaRotation2, relativePosition[1]);
		deltaVelocity += glm::dot(rotationVelocity2, this->contactNormal);
		deltaVelocity += this->body[1]->inv_mass;
	}
	//====== 所需冲量 ======
	float ratio = this->desiredDeltaVelocity / deltaVelocity;
	glm::vec3 impulseContact = glm::vec3(ratio, 0, 0);

	return impulseContact;
}

void Contact::ApplyVelocity(float dt) 
{
	//====== 期望速度变化 ======
	float velocityFromAcc = glm::dot(this->body[0]->acceleration * dt, this->contactNormal);
	if (this->body[1])
	{
		velocityFromAcc -= glm::dot(this->body[1]->acceleration * dt, this->contactNormal);
	}
	// If the velocity is very slow, limit the restitution
	real thisRestitution = restitution;
	if (real_abs(contactVelocity.x) < 0.25f)
		thisRestitution = (real)0.0f;
	this->desiredDeltaVelocity = -this->contactVelocity.x - thisRestitution * ((double)this->contactVelocity.x - velocityFromAcc);

	//无摩擦力
	glm::vec3 impulseContact = CalculateFrictionlessImpulse(dt);
	
	glm::vec3 impulse = this->contactToWorld * impulseContact;
	glm::vec3 impulsiveTorque = glm::cross(relativePosition[0], impulse);
	this->body[0]->velocity += this->body[0]->inv_mass * impulse;
	this->body[0]->rotation += this->body[0]->inverseInertiaTensorWorld * impulsiveTorque;

	/*int max = 20;
	if (this->body[0]->rotation.z > max) {
		this->body[0]->rotation.z = max;
	}
	else if(this->body[0]->rotation.z < -max){
		this->body[0]->rotation.z = -max;
	}*/
	//DebugerManager::PrintVec3(this->body[0]->rotation, "rotation: ");
}