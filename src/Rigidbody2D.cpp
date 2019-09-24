
#include <math.h>

#include "Rigidbody2D.h"
#include "DebugerManager.h"

using namespace My;

void Rigidbody2D::AddForce(glm::vec3 addForce)
{
	this->force = this->force + addForce;
}

void Rigidbody2D::ApplyTorque(glm::vec3 impulse)
{
	this->torque = this->torque + impulse;
}

//impulse: p = m * v
void Rigidbody2D::ApplyImpulse(glm::vec3 impulse)
{
	this->velocity = impulse * this->inv_mass;
}

void Rigidbody2D::CalcAllForce() 
{
	AddForce(this->m_gravity);
	//ApplyTorque(glm::vec3(0, 0, 40));
}

void Rigidbody2D::SetOrientation(const Quaternion &orientation)
{
	Rigidbody2D::orientation = orientation;
	Rigidbody2D::orientation.normalise();
}

void Rigidbody2D::SetOrientation(const double r, const double i,
	const double j, const double k)
{
	orientation.r = r;
	orientation.i = i;
	orientation.j = j;
	orientation.k = k;
	orientation.normalise();
}

void Rigidbody2D::GetOrientation(Quaternion *orientation) const
{
	*orientation = Rigidbody2D::orientation;
}

void Rigidbody2D::Integrate(float dt)
{
	if (this->isKinematic)
		return;
	
	CalcAllForce();
	
	//Update Linear Position
	this->UpdatePosition(this->velocity * dt);
	//liner velocity
	this->acceleration = this->force;
	this->velocity = this->velocity + this->acceleration * dt;
	//Impose Drag
	this->velocity *=  pow(this->linearDamping, dt);

	//Update Rotate
	glm::vec3 newAngularVec = this->angularVelocity * dt;
	//DebugerManager::PrintVec3(newAngularVec, "newAngularVec: ");
	this->UpdateRotate(newAngularVec.z);
	//力矩：M = r×F，r是力臂，F是外力
	//角加速度：a = M / I，I是转动惯量
	this->angularAcceleration = this->torque / 0.5f;
	this->angularVelocity = this->angularVelocity + this->angularAcceleration * dt;
	//Impose Drag
	this->angularVelocity *= pow(this->angularDamping, dt);

	//clear all forces
	this->force = glm::vec3(0,0,0);
	this->torque = glm::vec3(0, 0, 0);
}