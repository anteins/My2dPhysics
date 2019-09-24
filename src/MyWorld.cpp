#include <cmath>
#include<algorithm>

#include "MyWorld.h"
#include "MyGeometry.h"
#include "MyBox.h"
#include "MySphere.h"
#include "DebugerManager.h"

using namespace My;
using namespace std;

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);
bool Collide_Sphere_Box(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);
void ResolveCollision(Rigidbody2D& rigA, Rigidbody2D& rigB, float dt);

bool sortFun(Rigidbody2D* p1, Rigidbody2D* p2)
{
	return p1->GetId() < p2->GetId();//��������  
}

Rigidbody2D* MyWorld::CreateBox(glm::vec3 halfExtents)
{
	auto boxShape = make_shared<MyBox>(glm::vec3(0.0f), halfExtents);
	Rigidbody2D* body = new Rigidbody2D(boxShape);
	m_bodyList.push_back(body);
	//sort(m_bodyList.begin(), m_bodyList.end(), sortFun);
	return body;
}

Rigidbody2D* MyWorld::CreateSphere(float radius)
{
	auto sphere = make_shared<MySphere>(glm::vec3(0.0f), radius);
	Rigidbody2D* body = new Rigidbody2D(sphere);
	m_bodyList.push_back(body);
	//sort(m_bodyList.begin(), m_bodyList.end(), sortFun);
	return body;
}

void MyWorld::Step(float dt) 
{
	for (auto body : bodyList())
	{
		body->Integrate(dt);
	}

	ContactData contactData;
	for (size_t i = 0; i < bodyList().size(); ++i)
	{
		for (size_t j = i + 1; j < bodyList().size(); ++j) 
		{
			Rigidbody2D* body1 = bodyList()[i];
			Rigidbody2D* body2 = bodyList()[j];

			bool shouldCollide = this->Collide(body1, body2, contactData, dt);

			body1->isColliding = shouldCollide;
			//body2->isColliding = shouldCollide;

			if (shouldCollide) 
			{
				if (!contactData.body[0])
					return;

				Rigidbody2D* body = contactData.body[0];

				glm::vec3 relativeVelocity = body->velocity;
				if (contactData.body[1])
					relativeVelocity -= contactData.body[1]->velocity;
				float separatingVelocity = glm::dot(relativeVelocity, contactData.contactNormal);

				if (separatingVelocity > 0)
					return;

				float newSepVelocity = -separatingVelocity * contactData.restitution;

				glm::vec3 accCausedVelocity = body->acceleration;
				if (contactData.body[1])
					accCausedVelocity -= contactData.body[1]->acceleration;
				float accCausedSepVelocity = glm::dot(accCausedVelocity, contactData.contactNormal) * dt;
				printf("accCausedSepVelocity: %d\n", accCausedSepVelocity);

				if (accCausedSepVelocity < 0)
				{
					newSepVelocity += contactData.restitution * accCausedSepVelocity;

					if (newSepVelocity < 0) newSepVelocity = 0;
				}

				float deltaVelocity = newSepVelocity - separatingVelocity;

				float totalInverseMass = contactData.body[0]->inv_mass;
				if (contactData.body[1]) totalInverseMass += contactData.body[1]->inv_mass;

				if (totalInverseMass <= 0) return;

				// Calculate the impulse to apply
				float impulse = deltaVelocity / totalInverseMass;

				// Find the amount of impulse per unit of inverse mass
				glm::vec3 impulsePerIMass = contactData.contactNormal * impulse;

				contactData.body[0]->SetVelocity(contactData.body[0]->velocity + impulsePerIMass * contactData.body[0]->inv_mass);

				/*body->ApplyImpulse(newSepVelocity);
				body->ApplyTorque(newSepVelocity);*/
			}
		}
	}
}

void ResolveCollision(Rigidbody2D& rigA, Rigidbody2D& rigB, float dt)
{
	std::shared_ptr<MyGeometry> collisionA = rigA.GetShape();
	std::shared_ptr<MyGeometry> collisionB = rigB.GetShape();

	//rigB.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 normal = glm::vec3(0.0f, -1.0f, 0.0f);
	// Calculate relative velocity
	glm::vec3 rv = rigB.velocity - rigA.velocity;
	// Calculate relative velocity in terms of the normal direction
	float velAlongNormal = glm::dot(rv, normal);
	// Do not resolve if velocities are separating
	if (velAlongNormal > 0)
		return;
	
	//�ص�ϵ��
	rigA.restitution = 2.0f;
	rigB.restitution = 2.0f;

	// Calculate restitution
	float e = min(rigA.restitution, rigB.restitution);

	// Calculate impulse scalar
	float j = -(1 + e) * velAlongNormal;
	j /= 1 / rigA.mass + 1 / rigB.mass;

	// Apply impulse
	glm::vec3 impulse = j * normal;
	//rigA.ApplyImpulse(impulse);
	//rigB.velocity += 1 / rigB.mass * impulse;
	float mass_sum = rigA.mass + rigB.mass;
	float ratio = rigA.mass / mass_sum;
	rigA.velocity -= ratio * impulse;
	ratio = rigB.mass / mass_sum;
	rigB.velocity += ratio * impulse;

	//glm::vec3 force = glm::vec3(0, 9.8, 0);
	//rigA.AddForce(force);
	//printf("AddForce: (%f, %f, %f)\n", force.x, force.y, force.z);
}

bool MyWorld::Collide(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData, float dt)
{
	GeometryType gType = body1->GetShape()->GetGeometryType();
	GeometryType gType2 = body2->GetShape()->GetGeometryType();

	bool flag = false;
	if (gType == kBox && gType2 == kSphere) 
	{
		flag = Collide_Sphere_Box(body2, body1, cData);
	}
	else if (gType == kSphere && gType2 == kBox) 
	{
		flag = Collide_Sphere_Box(body1, body2, cData);
	}
	else 
	{
		flag = Collide_AABB(body1, body2, cData);
	}

	if (flag) 
	{
		//DebugerManager::DrawLine(body2->transform()->GetWorldPos(), body1->transform()->GetWorldPos(), DebugerManager::Color_Red);
	}
	return flag;
}

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData)
{
	AabbBound bound1 = body1->GetAabbBound();
	AabbBound bound2 = body2->GetAabbBound();

	if (bound1.min.x < bound2.max.x && bound1.max.x > bound2.min.x &&
		bound1.min.y < bound2.max.y && bound1.max.y > bound2.min.y) 
	{
		/*cData.pentration = -ballDistance;
		cData.contactPoint = spherePos - normalizeDirection * (ballDistance + radius);*/
		cData.contactNormal = -body1->velocity;
		return true;
	}
	return false;
}

bool Collide_Sphere_Box(Rigidbody2D* sphere, Rigidbody2D* box, ContactData& cData)
{
	std::shared_ptr<MySphere> sphereShape = std::static_pointer_cast<MySphere>(sphere->GetShape());
	std::shared_ptr<MyBox> boxShape = std::static_pointer_cast<MyBox>(box->GetShape());

	glm::vec3 spherePos = sphere->transform()->GetWorldPos();
	glm::vec3 boxPos = box->transform()->GetWorldPos();

	float radius = sphereShape->GetRadius();

	glm::vec3 boxDirection = boxShape->GetDirection();
	glm::vec3 normalizeDirection = glm::normalize(boxDirection);
	float ballDistance = glm::dot(spherePos, normalizeDirection) - radius - (boxShape->GetHalfExtents().y/2);
	if (ballDistance >= 0 || -ballDistance >= radius)
		return false;

	cData.pentration = -ballDistance;
	cData.contactPoint = spherePos - normalizeDirection * (ballDistance + radius);
	cData.contactNormal = boxDirection;
	cData.body[0] = sphere;
	cData.body[1] = nullptr;
	cData.restitution = 0.4f;

	/*DebugerManager::DrawVector3((normalizeDirection * (float)cData.pentration), body2, DebugerManager::Color_Green);
	DebugerManager::DrawPoint(cData.contactPoint, DebugerManager::Color_Green);*/
	DebugerManager::DrawVector3(cData.contactNormal, box, DebugerManager::Color_Blue);
	return true;
}

bool Collide_SAT(Rigidbody2D* body1, Rigidbody2D* body2) 
{
	return false;
}