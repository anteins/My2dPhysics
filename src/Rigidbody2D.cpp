
#include <math.h>

#include "Rigidbody2D.h"
#include "DebugerManager.h"

using namespace My;

/**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 */
static inline void _transformInertiaTensor(glm::mat3& iitWorld,
	const Quaternion& q,
	const glm::mat3& iitBody,
	const glm::mat4& rotmat)
{
	float t4 = rotmat[0][0] * iitBody[0][0] +
		rotmat[0][1] * iitBody[1][0] +
		rotmat[0][2] * iitBody[2][0];
	float t9 = rotmat[0][0] * iitBody[0][1] +
		rotmat[0][1] * iitBody[1][1] +
		rotmat[0][2] * iitBody[2][1];
	float t14 = rotmat[0][0] * iitBody[0][2] +
		rotmat[0][1] * iitBody[1][2] +
		rotmat[0][2] * iitBody[2][2];
	float t28 = rotmat[1][0] * iitBody[0][0] +
		rotmat[1][1] * iitBody[1][0] +
		rotmat[1][2] * iitBody[2][0];
	float t33 = rotmat[1][0] * iitBody[0][1] +
		rotmat[1][1] * iitBody[1][1] +
		rotmat[1][2] * iitBody[2][1];
	float t38 = rotmat[1][0] * iitBody[0][2] +
		rotmat[1][1] * iitBody[1][2] +
		rotmat[1][2] * iitBody[2][2];
	float t52 = rotmat[2][0] * iitBody[0][0] +
		rotmat[2][1] * iitBody[1][0] +
		rotmat[2][2] * iitBody[2][0];
	float t57 = rotmat[2][0] * iitBody[0][1] +
		rotmat[2][1] * iitBody[1][1] +
		rotmat[2][2] * iitBody[2][1];
	float t62 = rotmat[2][0] * iitBody[0][2] +
		rotmat[2][1] * iitBody[1][2] +
		rotmat[2][2] * iitBody[2][2];

	iitWorld[0][0] = t4 * rotmat[0][0] +
		t9 * rotmat[0][1] +
		t14 * rotmat[0][2];
	iitWorld[0][1] = t4 * rotmat[1][0] +
		t9 * rotmat[1][1] +
		t14 * rotmat[1][2];
	iitWorld[0][2] = t4 * rotmat[2][0] +
		t9 * rotmat[2][1] +
		t14 * rotmat[2][2];
	iitWorld[1][0] = t28 * rotmat[0][0] +
		t33 * rotmat[0][1] +
		t38 * rotmat[0][2];
	iitWorld[1][1] = t28 * rotmat[1][0] +
		t33 * rotmat[1][1] +
		t38 * rotmat[1][2];
	iitWorld[1][2] = t28 * rotmat[2][0] +
		t33 * rotmat[2][1] +
		t38 * rotmat[2][2];
	iitWorld[2][0] = t52 * rotmat[0][0] +
		t57 * rotmat[0][1] +
		t62 * rotmat[0][2];
	iitWorld[2][1] = t52 * rotmat[1][0] +
		t57 * rotmat[1][1] +
		t62 * rotmat[1][2];
	iitWorld[2][2] = t52 * rotmat[2][0] +
		t57 * rotmat[2][1] +
		t62 * rotmat[2][2];
}

static inline void _calculateTransformMatrix(glm::mat4& transformMatrix,
	const glm::vec3& position,
	const Quaternion& orientation)
{
	/*transformMatrix[0] = 1 - 2 * orientation.j * orientation.j -
		2 * orientation.k * orientation.k;
	transformMatrix[1] = 2 * orientation.i * orientation.j -
		2 * orientation.r * orientation.k;
	transformMatrix[2] = 2 * orientation.i * orientation.k +
		2 * orientation.r * orientation.j;
	transformMatrix[3] = position.x;

	transformMatrix[4] = 2 * orientation.i * orientation.j +
		2 * orientation.r * orientation.k;
	transformMatrix[5] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.k * orientation.k;
	transformMatrix[6] = 2 * orientation.j * orientation.k -
		2 * orientation.r * orientation.i;
	transformMatrix[7] = position.y;

	transformMatrix[8] = 2 * orientation.i * orientation.k -
		2 * orientation.r * orientation.j;
	transformMatrix[9] = 2 * orientation.j * orientation.k +
		2 * orientation.r * orientation.i;
	transformMatrix[10] = 1 - 2 * orientation.i * orientation.i -
		2 * orientation.j * orientation.j;
	transformMatrix[11] = position.z;*/
}

void Rigidbody2D::CalculateDerivedData()
{
	//orientation.normalise();

	// Calculate the transform matrix for the body.
	//_calculateTransformMatrix(transformMatrix, position, orientation);

	// Calculate the inertiaTensor in world space.
	_transformInertiaTensor(inverseInertiaTensorWorld,
		orientation,
		inverseInertiaTensor,
		this->m_motionState->GetMat44());

}

void Rigidbody2D::AddForce(glm::vec3 addForce)
{
	this->force = this->force + addForce;
}

void Rigidbody2D::AddTorque(glm::vec3 impulse)
{
	this->torque = this->torque + impulse;
}

//impulse: p = m * v
void Rigidbody2D::ApplyImpulse(glm::vec3 impulse)
{
	this->velocity += this->inv_mass * impulse;
}

void Rigidbody2D::ApplyTorqueImpulse(glm::vec3 impulse)
{
	this->rotation += this->inverseInertiaTensorWorld * impulse;
}

void Rigidbody2D::CalcAllForce() 
{
	AddForce(this->m_gravity);
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
	this->SetPosition(this->velocity * dt);
	//liner velocity
	this->acceleration = this->force;
	this->velocity = this->velocity + this->acceleration * dt;
	//Impose Drag
	this->velocity *=  pow(this->linearDamping, dt);

	//Update Rotate
	//DebugerManager::PrintVec3(this->rotation, "rotation: ");
	this->SetRotate(this->rotation);
	//力矩：M = r×F，r是力臂，F是外力
	//角加速度：a = M / I，I是转动惯量
	this->angularAcceleration = inverseInertiaTensorWorld * this->torque;
	this->rotation = this->rotation + this->angularAcceleration * dt;
	//Impose Drag
	this->rotation *= pow(this->angularDamping, dt);

	this->CalculateDerivedData();

	//clear all forces
	this->force = glm::vec3(0,0,0);
	this->torque = glm::vec3(0, 0, 0);
}