#include "ContactData.h"
#include "DebugerManager.h"

using namespace My;

float test_data[16] = {
	75,
	-75,
	52.96,
	14.75,
	-48.87,
	-14.75,
	-2.949,
	0.15,
	0.14,
	0.13,
	0.11,
	0.08,
	0.057,
	0.021,
	-0.015,
	-0.049,
};

int testIndex = 0;
void ContactData::CalculateInternals(float dt) 
{
	if (!this->body[0])
		return;

	Rigidbody2D* body1 = this->body[0];
	Rigidbody2D* body2 = this->body[1];

	//create contact basic
	CreateContactBasic();

	//碰撞点相对原点位置 q_rel
	glm::vec3 relativePosition[2];
	glm::vec3 tmp = this->body[0]->GetPosition();
	relativePosition[0] = this->contactPoint - tmp;
	if(this->body[1])
		relativePosition[1] = this->contactPoint - this->body[1]->GetPosition();

	//1、计算闭合速度(仅碰撞法线上)
	glm::vec3 contactVelocity;
	glm::vec3 rotation0 = this->body[0]->GetRotate();
	contactVelocity = glm::cross(rotation0, relativePosition[0]);
	contactVelocity += this->body[0]->GetVelocity();
	contactVelocity = this->worldToContact * contactVelocity;

	glm::vec3 accVelocity = this->body[0]->acceleration * dt;
	accVelocity = this->worldToContact * accVelocity;
	accVelocity.x = 0;
	contactVelocity += accVelocity;

	if (this->body[1]) 
	{
		glm::vec3 contactVelocity2;
		contactVelocity2 = glm::cross(this->body[1]->GetRotate(), relativePosition[1]);
		contactVelocity2 += this->body[1]->GetVelocity();
		contactVelocity2 = this->worldToContact * contactVelocity2;

		glm::vec3 accVelocity2 = this->body[1]->acceleration * dt;
		accVelocity2 = this->worldToContact * accVelocity2;
		accVelocity2.x = 0;
		contactVelocity2 += accVelocity2;

		contactVelocity -= contactVelocity2;
	}

	//2、计算期望速度变化
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

	this->desiredDeltaVelocity = -contactVelocity.x - thisRestitution * ((double)contactVelocity.x - accCausedSepVelocity);

	//处理相交
	glm::vec3 tmpPos = this->body[0]->GetPosition();
	//test data
	this->penetration = 0.08f;
	glm::vec3 move = this->penetration * this->contactNormal;
	this->body[0]->SetPosition(move);

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
	//test data
	impulseContact = glm::vec3(10,0,0);

	glm::vec3 impulse = this->contactToWorld * impulseContact;

	glm::vec3 impulsiveTorque = glm::cross(relativePosition[0], impulse);

	this->body[0]->velocity += this->body[0]->inv_mass * impulse;
	this->body[0]->rotation += this->body[0]->inverseInertiaTensorWorld * impulsiveTorque;

	//this->body[0]->ApplyImpulse(impulse);
	//this->body[0]->ApplyTorqueImpulse(impulsiveTorque);
}

void ContactData::CreateContactBasic() 
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