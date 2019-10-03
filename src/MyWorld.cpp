#include <cmath>
#include<algorithm>

#include "MyWorld.h"
#include "MyGeometry.h"
#include "MyBox.h"
#include "MySphere.h"
#include "DebugerManager.h"

using namespace My;
using namespace std;

bool Collide_SAT(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);
bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);
bool Collide_Sphere_Box(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);
bool Collide_Sphere_Sphere(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData);

void ResolveContacts(std::vector<ContactData>& contactList, float dt);
void ResolveContacts_Particle(std::vector<ContactData>& contactList, float dt);
void ResolvePenetration_Particle(std::vector<ContactData>& contactList, float dt);

Rigidbody2D* MyWorld::CreateBox(glm::vec3 halfExtents)
{
	auto boxShape = make_shared<MyBox>(glm::vec3(0.0f), halfExtents);
	Rigidbody2D* body = new Rigidbody2D(boxShape);
	m_bodyList.push_back(body);
	return body;
}

Rigidbody2D* MyWorld::CreateSphere(float radius)
{
	auto sphere = make_shared<MySphere>(glm::vec3(0.0f), radius);
	Rigidbody2D* body = new Rigidbody2D(sphere);
	m_bodyList.push_back(body);
	return body;
}

void MyWorld::Step(float dt) 
{
	for (auto body : bodyList())
	{
		body->Integrate(dt);
	}

	std::vector<ContactData> contactList;
	for (size_t i = 0; i < bodyList().size(); ++i)
	{
		for (size_t j = i + 1; j < bodyList().size(); ++j) 
		{
			Rigidbody2D* body1 = bodyList()[i];
			Rigidbody2D* body2 = bodyList()[j];

			ContactData contactData;
			bool shouldCollide = this->Collide(body1, body2, contactData, dt);

			body1->isColliding = shouldCollide;
			body2->isColliding = shouldCollide;

			if (shouldCollide)
			{
				contactList.push_back(contactData);
			}
		}
	}

	ResolveContacts_Particle(contactList, dt);
	ResolvePenetration_Particle(contactList, dt);
}

bool MyWorld::Collide(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData, float dt)
{
	GeometryType gType1 = body1->GetShape()->GetGeometryType();
	GeometryType gType2 = body2->GetShape()->GetGeometryType();

	bool flag = false;
	if (gType1 == kBox && gType2 == kSphere) 
	{
		flag = Collide_Sphere_Box(body2, body1, cData);
	}
	else if (gType1 == kSphere && gType2 == kBox) 
	{
		flag = Collide_Sphere_Box(body1, body2, cData);
	}
	else if (gType1 == kSphere && gType2 == kSphere) 
	{
		flag = Collide_Sphere_Sphere(body1, body2, cData);
	}
	else 
	{
		//flag = Collide_AABB(body1, body2, cData);
		flag = Collide_SAT(body1, body2, cData);
	}

	return flag;
}

void ResolveContacts_Particle(std::vector<ContactData>& contactList, float dt)
{
	for (size_t i = 0; i < contactList.size(); ++i)
	{
		ContactData contactData = contactList[i];

		if (!contactData.body[0])
			continue;

		Rigidbody2D* body1 = contactData.body[0];
		Rigidbody2D* body2 = contactData.body[1];

		// separatingVelocity
		glm::vec3 relativeVelocity = glm::vec3(0.0f);
		if (body2)
		{
			relativeVelocity = body1->velocity - body2->velocity;
		}
		else
		{
			relativeVelocity = body1->velocity;
		}
		float separatingVelocity = glm::dot(relativeVelocity, contactData.contactNormal);

		//两个物体处于分离中的状态，无须计算碰撞冲量
		if (separatingVelocity > 0)
			return;

		float newSepVelocity = -separatingVelocity * contactData.restitution;

		glm::vec3 accCausedVelocity(0);
		if (body2)
		{
			accCausedVelocity = body1->acceleration - body2->acceleration;
		}
		else
		{
			accCausedVelocity = body1->acceleration;
		}
		float accCausedSepVelocity = glm::dot(accCausedVelocity, contactData.contactNormal) * dt;

		//std::printf("accCausedSepVelocity: %f\n", accCausedSepVelocity);

		if (accCausedSepVelocity < 0)
		{
			newSepVelocity += contactData.restitution * accCausedSepVelocity;

			if (newSepVelocity < 0) newSepVelocity = 0;
		}

		float deltaVelocity = newSepVelocity - separatingVelocity;

		float totalInverseMass = body1->inv_mass;
		if (body2)
			totalInverseMass += body2->inv_mass;

		if (totalInverseMass <= 0) return;

		// Calculate the impulse to apply
		float impulse = deltaVelocity / totalInverseMass;

		// Find the amount of impulse per unit of inverse mass
		glm::vec3 impulsePerIMass = contactData.contactNormal * impulse;

		body1->SetVelocity(body1->velocity + impulsePerIMass * body1->inv_mass);

		// Particle 1 goes in the opposite direction
		if (body2) {
			body2->SetVelocity(body2->velocity +
				impulsePerIMass * -body2->inv_mass
			);
		}
	}
}

void ResolvePenetration_Particle(std::vector<ContactData>& contactList, float dt)
{
	for (size_t i = 0; i < contactList.size(); ++i) 
	{
		ContactData contactData = contactList[i];

		//printf("contactData.penetration: %d \n", contactData.penetration);

		if (contactData.penetration <= 0)
			continue;

		

		if (!contactData.body[0] || !contactData.body[1])
			continue;

		Rigidbody2D* body1 = contactData.body[0];
		Rigidbody2D* body2 = contactData.body[1];

		float totalInvMass = body1->inv_mass + body2->inv_mass;

		float penetration_1 = body2->mass * totalInvMass * contactData.penetration;
		float penetration_2 = body1->mass * totalInvMass * contactData.penetration;

		glm::vec3 toVec_1 = penetration_1 * contactData.contactNormal;
		glm::vec3 toVec_2 = -penetration_2 * contactData.contactNormal;

		body1->SetPosition(body1->GetPosition() + toVec_1);
		body2->SetPosition(body2->GetPosition() + toVec_2);
	}
}

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, ContactData& cData)
{
	AabbBound bound1 = body1->GetAabbBound();
	AabbBound bound2 = body2->GetAabbBound();

	if (bound1.min.x < bound2.max.x && bound1.max.x > bound2.min.x &&
		bound1.min.y < bound2.max.y && bound1.max.y > bound2.min.y) 
	{
		/*cData.penetration = -ballDistance;
		cData.contactPoint = spherePos - normalizeDirection * (ballDistance + radius);*/
		//cData.contactNormal = glm::vec3(0) - body1->velocity;
		return true;
	}
	return false;
}

///
bool Collide_Sphere_Box(Rigidbody2D* sphere, Rigidbody2D* box, ContactData& cData)
{
	std::shared_ptr<MySphere> sphereShape = std::static_pointer_cast<MySphere>(sphere->GetShape());
	std::shared_ptr<MyBox> boxShape = std::static_pointer_cast<MyBox>(box->GetShape());

	glm::vec3 spherePos = sphere->transform()->GetWorldPos();
	glm::vec3 boxPos = box->transform()->GetWorldPos();

	float radius = sphereShape->GetRadius();

	glm::vec3 boxDirection = boxShape->GetDirection();
	glm::vec3 normalizeDirection = boxDirection;

	//DebugerManager::DrawVector3(boxPos, boxPos + normalizeDirection, DebugerManager::Color_Green);

	float ballDistance = glm::dot(spherePos, normalizeDirection) - radius - (boxShape->GetHalfExtents().y/2);
	if (ballDistance >= 0)
		return false;

	cData.penetration = -ballDistance;
	cData.contactPoint = spherePos - normalizeDirection * (ballDistance + radius);
	cData.contactNormal = boxDirection;
	cData.body[0] = sphere;
	cData.body[1] = nullptr;
	cData.restitution = 0.4f;

	return true;
}

bool Collide_Sphere_Sphere(Rigidbody2D* sphere1, Rigidbody2D* sphere2, ContactData& cData)
{
	std::shared_ptr<MySphere> sphere1Shape = std::static_pointer_cast<MySphere>(sphere1->GetShape());
	std::shared_ptr<MySphere> sphere2Shape = std::static_pointer_cast<MySphere>(sphere2->GetShape());

	glm::vec3 pos1 = sphere1->transform()->GetWorldPos();
	glm::vec3 pos2 = sphere2->transform()->GetWorldPos();

	float radius1 = sphere1Shape->GetRadius();
	float radius2 = sphere2Shape->GetRadius();

	float distance = glm::distance(pos1, pos2) - (radius1 + radius2);

	if (distance >= 0)
		return false;

	glm::vec3 contactNormal = glm::normalize(pos1 - pos2);
	glm::vec3 contactPoint = contactNormal * radius1;

	cData.penetration = -distance;
	cData.contactPoint = contactPoint;
	cData.contactNormal = contactNormal;
	cData.body[0] = sphere1;
	cData.body[1] = sphere2;
	cData.restitution = 0.4f;

	return true;
}

void projectPoint(glm::vec2& limit, glm::vec3 point, glm::vec3 axis)
{
	// Find the dot product
	float projection = (axis.x * point.x + axis.y * point.y) / (axis.x * axis.x + axis.y * axis.y);
	// Get the min and max values
	limit.x = std::min(projection, limit.x);
	limit.y = std::max(projection, limit.y);
}

#define BIG_NUM 9999999
bool Collide_SAT(Rigidbody2D* body_a, Rigidbody2D* body_b, ContactData& cData)
{
	std::shared_ptr<MyBox> shape_a = std::static_pointer_cast<MyBox>(body_a->GetShape());
	std::shared_ptr<MyBox> shape_b = std::static_pointer_cast<MyBox>(body_b->GetShape());

	glm::vec3 pos_a = body_a->transform()->GetWorldPos();
	glm::vec3 pos_b = body_b->transform()->GetWorldPos();

	//test data
	int count_a = 4;
	int count_b = 4;

	glm::mat4 transform_a = body_a->transform()->GetMatrix();
	glm::mat4 transform_b = body_b->transform()->GetMatrix();

	glm::vec3 overlap = glm::vec3(0);

	// Check pionts of shape b against axis of shape a
	for (unsigned int i = 0; i < count_a; i++) 
	{
		// Get face
		glm::vec4 point_a = transform_a * glm::vec4(shape_a->GetPoint(i), 1.0f);
		glm::vec4 point_b = transform_a * glm::vec4(shape_a->GetPoint((i + 1) % count_a), 1.0f);
		// Get projection axis
		glm::vec3 axis = point_a - point_b;
		axis = glm::vec3(-axis.y, axis.x, 0);
		glm::vec2 limit_a = glm::vec2(BIG_NUM, -BIG_NUM);
		glm::vec2 limit_b = glm::vec2(BIG_NUM, -BIG_NUM);
		// Project all points in shape a
		for (unsigned int j = 0; j < count_a; j++) 
		{
			projectPoint(limit_a, transform_a * glm::vec4(shape_a->GetPoint(j), 1.0f), axis);
		}
			
		// Project all points in shape b
		for (unsigned int j = 0; j < count_b; j++) 
		{
			projectPoint(limit_b, transform_b * glm::vec4(shape_b->GetPoint(j), 1.0f), axis);
		}
			
		// Overlap found, find solution 
		float delta = limit_b.x - limit_a.y;
		// No overlap found
		if (delta >= 0)
			return false;

		if (delta > overlap.z)
			overlap = glm::vec3(axis.x, axis.y, delta);
	}

	// Check pionts of shape a against axis of shape b
	for (unsigned int i = 0; i < count_b; i++)
	{
		// Get face
		glm::vec4 point_a = transform_b * glm::vec4(shape_b->GetPoint(i), 1.0f);
		glm::vec4 point_b = transform_b * glm::vec4(shape_b->GetPoint((i + 1) % count_b), 1.0f);
		// Get projection axis
		glm::vec3 axis = point_a - point_b;
		axis = glm::vec3(-axis.y, axis.x, 0);
		glm::vec2 limit_a = glm::vec2(BIG_NUM, -BIG_NUM);
		glm::vec2 limit_b = glm::vec2(BIG_NUM, -BIG_NUM);

		// Project all points in shape a
		for (unsigned int j = 0; j < count_a; j++)
			projectPoint(limit_a, transform_a * glm::vec4(shape_a->GetPoint(j), 1.0f), axis);
		// Project all points in shape b
		for (unsigned int j = 0; j < count_b; j++)
			projectPoint(limit_b, transform_b * glm::vec4(shape_b->GetPoint(j), 1.0f), axis);

		// Overlap found, find solution 
		float delta = limit_b.x - limit_a.y;
		// No overlap found
		if (delta >= 0)
			return false;

		if (delta > overlap.z)
			overlap = glm::vec3(axis.x, axis.y, delta);
	}
	
	glm::vec3 boxDirection = shape_b->GetDirection();
	glm::vec3 normalizeDirection = boxDirection;

	cData.penetration = glm::length(overlap);
	cData.contactPoint = transform_a * glm::vec4(shape_a->GetPoint(1), 1.0);
	cData.contactNormal = glm::normalize(boxDirection);
	cData.body[0] = body_a;
	cData.body[1] = nullptr;
	if(!body_b->IsKinematic())
	{
		cData.body[1] = body_b;
	}
	
	cData.restitution = 0.4f;

	//DebugerManager::DrawPoint(cData.contactPoint);
	//DebugerManager::DrawVector3(pos_a, pos_a + cData.contactNormal);

	return true;
}

//void ResolveCollision(Rigidbody2D& rigA, Rigidbody2D& rigB, float dt)
//{
//	std::shared_ptr<MyGeometry> collisionA = rigA.GetShape();
//	std::shared_ptr<MyGeometry> collisionB = rigB.GetShape();
//
//	//rigB.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
//	glm::vec3 normal = glm::vec3(0.0f, -1.0f, 0.0f);
//	// Calculate relative velocity
//	glm::vec3 rv = rigB.velocity - rigA.velocity;
//	// Calculate relative velocity in terms of the normal direction
//	float velAlongNormal = glm::dot(rv, normal);
//	// Do not resolve if velocities are separating
//	if (velAlongNormal > 0)
//		return;
//
//	//回弹系数
//	rigA.restitution = 2.0f;
//	rigB.restitution = 2.0f;
//
//	// Calculate restitution
//	float e = min(rigA.restitution, rigB.restitution);
//
//	// Calculate impulse scalar
//	float j = -(1 + e) * velAlongNormal;
//	j /= 1 / rigA.mass + 1 / rigB.mass;
//
//	// Apply impulse
//	glm::vec3 impulse = j * normal;
//	//rigA.ApplyImpulse(impulse);
//	//rigB.velocity += 1 / rigB.mass * impulse;
//	float mass_sum = rigA.mass + rigB.mass;
//	float ratio = rigA.mass / mass_sum;
//	rigA.velocity -= ratio * impulse;
//	ratio = rigB.mass / mass_sum;
//	rigB.velocity += ratio * impulse;
//
//	//glm::vec3 force = glm::vec3(0, 9.8, 0);
//	//rigA.AddForce(force);
//	//printf("AddForce: (%f, %f, %f)\n", force.x, force.y, force.z);
//}