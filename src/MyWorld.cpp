#include "MyWorld.h"
#include "MyBox.h"
#include "MySphere.h"
#include "DebugerManager.h"
#include <cmath>

using namespace My;
using namespace std;

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, float dt);
void ResolveCollision(Rigidbody2D* rigA, Rigidbody2D* rigB, float dt);

Rigidbody2D* MyWorld::CreateBox(glm::vec3 halfExtents)
{
	glm::vec3 centerPosition = glm::vec3(0.0f);
	auto collision = make_shared<MyBox>(centerPosition, halfExtents);
	Rigidbody2D* body = new Rigidbody2D(collision);
	m_bodyList.push_back(body);
	return body;
}

Rigidbody2D* MyWorld::CreateSphere(glm::vec3 centerPosition, float radius)
{
	auto collision = make_shared<MySphere>(centerPosition, radius);
	Rigidbody2D* body = new Rigidbody2D(collision);
	m_bodyList.push_back(body);
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

			bool shouldCollide = this->Collide(body1, body2, &contactData, dt);

			body1->isColliding = shouldCollide;
			//body2->isColliding = shouldCollide;

			if (shouldCollide) 
			{
				//debug
				/*body1->SetKinematic(true);
				body2->SetKinematic(true);
				DebugerManager::DrawVector3(body1->GetCollisionShape()->transform()->position(), body2->GetCollisionShape()->transform()->position(), DebugerManager::Color_Red);*/

				//DebugerManager::ClearDebugBuffers();
				//DebugerManager::DrawLine(body1->GetCollisionShape()->transform()->position(), body2->GetCollisionShape()->transform()->position(), DebugerManager::Color_Red);
				//ResolveCollision(body1, body2, dt);

				glm::vec3 contactNormal = glm::normalize(body1->GetCollisionShape()->transform()->position() - body2->GetCollisionShape()->transform()->position());
				/*DebugerManager::DrawVector3(body1->GetCollisionShape()->transform()->position(), body2->GetCollisionShape()->transform()->position(), DebugerManager::Color_Red);
				body1->SetKinematic(true);*/
				
				float restitution = 0.4f;
				glm::vec3 newSepVelocity = -body1->velocity * restitution;
				glm::vec3 accCausedVelocity1 = body1->velocity + glm::vec3(0.0f, -9.8f, 0.0f) * contactNormal * dt;//合外力的速度
				glm::vec3 accCausedVelocity2 = newSepVelocity;//碰撞后的速度

				float v1 = abs(accCausedVelocity2.y);
				float vg = abs(accCausedVelocity1.y);
				/*printf("-----------------------\n");
				printf("碰撞后的速度: %f,   合外力的速度: %f\n", v1, vg);*/
				if (v1 < vg)
				{
					body1->velocity = glm::vec3(0.0f);
				}
				else 
				{
					body1->ApplyImpulse(newSepVelocity);
				}
			}
		}
	}
}

void ResolveCollision(Rigidbody2D& rigA, Rigidbody2D& rigB, float dt)
{
	std::shared_ptr<MyGeometry> collisionA = rigA.GetCollisionShape();
	std::shared_ptr<MyGeometry> collisionB = rigB.GetCollisionShape();

	//rigB.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 normal = glm::vec3(0.0f, -1.0f, 0.0f);
	// Calculate relative velocity
	glm::vec3 rv = rigB.velocity - rigA.velocity;
	// Calculate relative velocity in terms of the normal direction
	float velAlongNormal = glm::dot(rv, normal);
	// Do not resolve if velocities are separating
	if (velAlongNormal > 0)
		return;
	
	//回弹系数
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

bool MyWorld::Collide(Rigidbody2D* body1, Rigidbody2D* body2, ContactData* contactData, float dt)
{
	bool flag = Collide_AABB(body1, body2, dt);
	if (flag) 
	{
		auto collsion1 = body1->GetCollisionShape();
		auto collsion2 = body2->GetCollisionShape();
		contactData->collisionNormal = collsion1->transform()->position() - collsion2->transform()->position();
	}
	return flag;
}

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, float dt) 
{
	AabbBound bound1 = body1->GetCollisionShape()->GetBound();
	AabbBound bound2 = body2->GetCollisionShape()->GetBound();

	if (bound1.min.x < bound2.max.x && bound1.max.x > bound2.min.x && 
		bound1.min.y < bound2.max.y && bound1.max.y > bound2.min.y)
		return true;

	return false;
}