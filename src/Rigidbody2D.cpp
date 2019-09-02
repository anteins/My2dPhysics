
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
	AddForce(this->_gravity);
	//ApplyTorque(glm::vec3(0, 0, 40));
}

void Rigidbody2D::Integrate(float dt)
{
	if (this->isKinematic)
		return;
	
	auto collision = this->GetCollisionShape();
	CalcAllForce();
	
	//Update
	collision->UpdatePosition(velocity * dt);
	//liner velocity
	this->acceleration = this->force;
	this->velocity = this->velocity + this->acceleration * dt;
	//Impose Drag
	this->velocity = this->velocity * this->linearDamping;

	//Update
	collision->UpdateRotate(this->angularVelocity * dt);
	//力矩：M = r×F，r是力臂，F是外力
	//角加速度：a = M / I，I是转动惯量
	this->angularAcceleration = this->torque / 0.5f;
	this->angularVelocity = this->angularVelocity + this->angularAcceleration * dt;
	//Impose Drag
	this->angularVelocity = this->angularVelocity * this->angularDamping;

	//clear all forces
	this->force = glm::vec3(0,0,0);
	this->torque = glm::vec3(0, 0, 0);
}