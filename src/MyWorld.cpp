#include <cmath>
#include <algorithm>

#include "MyWorld.h"
#include "MyGeometry.h"
#include "MyBox.h"
#include "MySphere.h"
#include "DebugerManager.h"

using namespace My;
using namespace std;

bool Collide_SAT(Rigidbody2D* body1, Rigidbody2D* body2, CollisionData* data);
bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, Contact& cData);
bool Collide_Sphere_Panel(Rigidbody2D* body1, Rigidbody2D* body2, Contact& cData);
bool Collide_Box_Sphere(Rigidbody2D* box, Rigidbody2D* sphere, Contact& cData);
bool Collide_Sphere_Sphere(Rigidbody2D* body1, Rigidbody2D* body2, Contact& cData);

void ResolveContacts(Contact* contacts,int numContacts,float dt);
void ResolveContacts_Particle(std::vector<Contact>& contactList, float dt);
void ResolvePenetration_Particle(std::vector<Contact>& contactList, float dt);

Rigidbody2D* MyWorld::CreateBox(glm::vec3 halfExtents)
{
	auto shape_box = make_shared<MyBox>(glm::vec3(0.0f), halfExtents);
	Rigidbody2D* body = new Rigidbody2D(shape_box);
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

	this->cData.reset(10);
	this->cData.friction = 0.9f;
	this->cData.restitution = 0.4f;
	this->cData.tolerance = 0.1f;

	for (size_t i = 0; i < bodyList().size(); ++i)
	{
		if (!cData.hasMoreContacts()) return;

		for (size_t j = i + 1; j < bodyList().size(); ++j) 
		{
			if (!cData.hasMoreContacts()) return;

			Rigidbody2D* body1 = bodyList()[i];
			Rigidbody2D* body2 = bodyList()[j];
			bool shouldCollide = this->Collide(body1, body2, this->cData, dt);

			body1->isColliding = shouldCollide;
			body2->isColliding = shouldCollide;
		}
	}

	//Particle
	//ResolveContacts_Particle(contactList, dt);
	//ResolvePenetration_Particle(contactList, dt);

	//RigidBody
	ResolveContacts(
		cData.contactArray,
		cData.contactCount,
		dt
	);
}

bool MyWorld::Collide(Rigidbody2D* body1, Rigidbody2D* body2, CollisionData& cData, float dt)
{
	GeometryType gType1 = body1->GetShape()->GetGeometryType();
	GeometryType gType2 = body2->GetShape()->GetGeometryType();

	bool flag = false;
	if (gType1 == kBox && gType2 == kSphere) 
	{
		//flag = Collide_Box_Sphere(body2, body1, cData);
	}
	else if (gType1 == kSphere && gType2 == kBox) 
	{
		//flag = Collide_Box_Sphere(body2, body1, cData);
	}
	else if (gType1 == kSphere && gType2 == kSphere) 
	{
		//flag = Collide_Sphere_Sphere(body1, body2, cData);
	}
	else 
	{
		//flag = Collide_AABB(body1, body2, cData);
		flag = Collide_SAT(body1, body2, &cData);
	}

	return flag;
}

//刚体-碰撞响应
void ResolveContacts(
	Contact* contacts, 
	int numContacts,
	float dt)
{
	if (numContacts == 0) return;

	Contact* lastContact = contacts + numContacts;
	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		contact->CalculateInternals(dt);
	}

	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		contact->ApplyPosition(dt);
	}

	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		contact->ApplyVelocity(dt);
	}
}

//粒子-碰撞响应
void ResolveContacts_Particle(std::vector<Contact>& contactList, float dt)
{
	for (size_t i = 0; i < contactList.size(); ++i)
	{
		Contact contactData = contactList[i];

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

void ResolvePenetration_Particle(std::vector<Contact>& contactList, float dt)
{
	for (size_t i = 0; i < contactList.size(); ++i) 
	{
		Contact contactData = contactList[i];

		if (contactData.penetration <= 0)
			continue;

		Rigidbody2D* body1 = contactData.body[0];
		Rigidbody2D* body2 = contactData.body[1];

		if (body2 == nullptr)
			continue;

		glm::vec3 pos_a = body1->GetPosition();
		glm::vec3 pos_b = body2->GetPosition();

		//printf("contactData.penetration: %d \n", contactData.penetration);

		float totalInvMass = body1->inv_mass + body2->inv_mass;

		glm::vec3 contactNormal = glm::normalize(pos_a - pos_b);
		glm::vec3 movement = contactNormal * contactData.penetration;
		glm::vec3 toVec_1 = body2->mass / totalInvMass * movement;
		body1->SetPosition(body1->GetPosition() + toVec_1);

		glm::vec3 toVec_2 = body1->mass / totalInvMass * -movement;
		body2->SetPosition(body2->GetPosition() + toVec_2);
	}
}

bool Collide_AABB(Rigidbody2D* body1, Rigidbody2D* body2, Contact& cData)
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

bool Collide_Sphere_Panel(Rigidbody2D* sphere, Rigidbody2D* box, Contact& cData)
{
	std::shared_ptr<MySphere> shape_sphere = std::static_pointer_cast<MySphere>(sphere->GetShape());
	std::shared_ptr<MyBox> shape_box = std::static_pointer_cast<MyBox>(box->GetShape());

	glm::vec3 spherePos = sphere->GetPosition();
	glm::vec3 boxPos = box->GetPosition();

	float radius = shape_sphere->GetRadius();

	glm::vec3 boxDirection = shape_box->GetDirection();
	glm::vec3 normalizeDirection = boxDirection;

	float ballDistance = glm::dot(spherePos, normalizeDirection) - radius - (shape_box->GetHalfExtents().y/2);
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

bool Collide_Box_Sphere(Rigidbody2D* box, Rigidbody2D* sphere, Contact& cData)
{
	std::shared_ptr<MyBox> shape_box = std::static_pointer_cast<MyBox>(box->GetShape());
	std::shared_ptr<MySphere> shape_sphere = std::static_pointer_cast<MySphere>(sphere->GetShape());

	// Transform the centre of the sphere into box coordinates
	glm::vec3 centre = sphere->GetPosition();
	glm::mat4 tmpMat = box->GetTransform().GetMat44();
	glm::mat4 tmpMatInv = glm::inverse(tmpMat);
	glm::vec3 relCentre = tmpMatInv * glm::vec4(centre, 1.0f);

	float radius = shape_sphere->GetRadius();
	glm::vec3 halfSize = shape_box->GetHalfExtent();
	halfSize = glm::vec3(halfSize.x / 2, halfSize.y / 2, halfSize.z / 2);
	bool b1 = real_abs(relCentre.x) - radius > halfSize.x;
	bool b2 = real_abs(relCentre.y) - radius > halfSize.y;
	bool b3 = real_abs(relCentre.z) - radius > halfSize.z;

	// Early out check to see if we can exclude the contact
	if (b1 ||b2 ||b3)
	{
		return false;
	}

	glm::vec3 closestPt(0, 0, 0);
	real dist;

	// Clamp each coordinate to the box.
	dist = relCentre.x;
	if (dist > halfSize.x) dist = halfSize.x;
	if (dist < -halfSize.x) dist = -halfSize.x;
	closestPt.x = dist;

	dist = relCentre.y;
	if (dist > halfSize.y) dist = halfSize.y;
	if (dist < -halfSize.y) dist = -halfSize.y;
	closestPt.y = dist;

	dist = relCentre.z;
	if (dist > halfSize.z) dist = halfSize.z;
	if (dist < -halfSize.z) dist = -halfSize.z;
	closestPt.z = dist;

	// Check we're in contact
	glm::vec3 tmp = (closestPt - relCentre);
	dist = tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z;//squareMagnitude();
	if (dist > radius * radius) 
		return false;

	// Compile the contact
	glm::vec3 closestPtWorld = tmpMat * glm::vec4(closestPt, 1.0f);

	cData.contactNormal = glm::normalize(centre - closestPtWorld);
	cData.contactPoint = closestPtWorld;
	cData.penetration = radius - real_sqrt(dist);
	cData.body[0] = sphere;
	cData.body[1] = nullptr;
	cData.restitution = 0.4f;

	return true;
}

bool Collide_Sphere_Sphere(Rigidbody2D* sphere1, Rigidbody2D* sphere2, Contact& cData)
{
	std::shared_ptr<MySphere> sphere1Shape = std::static_pointer_cast<MySphere>(sphere1->GetShape());
	std::shared_ptr<MySphere> sphere2Shape = std::static_pointer_cast<MySphere>(sphere2->GetShape());

	glm::vec3 pos1 = sphere1->GetPosition();
	glm::vec3 pos2 = sphere2->GetPosition();

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

void projectPoint(glm::vec2& limit, glm::vec3 point, glm::vec2 axis)
{
	// Find the dot product
	float projection = (axis.x * point.x + axis.y * point.y) / (axis.x * axis.x + axis.y * axis.y);
	// Get the min and max values
	limit.x = std::min(projection, limit.x);
	limit.y = std::max(projection, limit.y);
}

glm::vec2 getAxis(glm::vec3 point_a, glm::vec3 point_b)
{
	glm::vec2 axis = point_a - point_b;
	// Make it a unit vector
	float unit_len = std::sqrt(axis.x * axis.x + axis.y * axis.y);
	axis.x /= unit_len;
	axis.y /= unit_len;

	return axis;
}

class DeepthInfo {
public:
	unsigned int index;
	float projection;

	DeepthInfo(unsigned int index, float projection) { this->index = index; this->projection = projection; }
};


//自定义排序函数
bool SortByProjection(const DeepthInfo& v1, const DeepthInfo& v2)
{
	return v1.projection < v2.projection;//升序排列
}

#define BIG_NUM 9999999
bool Collide_SAT(Rigidbody2D* body_a, Rigidbody2D* body_b, CollisionData* cData)
{
	//test data
	int count_a = 4;
	int count_b = 4;

	glm::mat4 transform_a = body_a->GetMat44();
	glm::mat4 transform_b = body_b->GetMat44();

	glm::vec3 pos_a = body_a->GetPosition();
	glm::vec3 pos_b = body_b->GetPosition();

	std::shared_ptr<MyBox> shape_a = std::static_pointer_cast<MyBox>(body_a->GetShape());
	std::shared_ptr<MyBox> shape_b = std::static_pointer_cast<MyBox>(body_b->GetShape());

	glm::vec4 points_a[4];
	points_a[0] = transform_a * glm::vec4(shape_a->GetPoint(0), 1.0f);
	points_a[1] = transform_a * glm::vec4(shape_a->GetPoint(1), 1.0f);
	points_a[2] = transform_a * glm::vec4(shape_a->GetPoint(2), 1.0f);
	points_a[3] = transform_a * glm::vec4(shape_a->GetPoint(3), 1.0f);

	glm::vec4 points_b[4];
	points_b[0] = transform_b * glm::vec4(shape_b->GetPoint(0), 1.0f);
	points_b[1] = transform_b * glm::vec4(shape_b->GetPoint(1), 1.0f);
	points_b[2] = transform_b * glm::vec4(shape_b->GetPoint(2), 1.0f);
	points_b[3] = transform_b * glm::vec4(shape_b->GetPoint(3), 1.0f);

	glm::vec3 overlap = glm::vec3(0, 0, -std::numeric_limits<float>::max());

	// Check pionts of shape b against axis of shape a
	//printf("=========================== AAA (%f, %f) ===========================\n", pos_a.x, pos_a.y);
	for (unsigned int i = 0; i < count_a; i++)
	{
		// Get face
		glm::vec4 point_a = points_a[i];
		glm::vec4 point_b = points_a[(i + 1) % count_a];
		// Get projection axis
		glm::vec2 axis = getAxis(point_a, point_b);
		axis = glm::vec2(-axis.y, axis.x);
		glm::vec2 limit_a = glm::vec2(BIG_NUM, -BIG_NUM);
		glm::vec2 limit_b = glm::vec2(BIG_NUM, -BIG_NUM);
		// Project all points in shape a
		for (unsigned int j = 0; j < count_a; j++)
		{
			projectPoint(limit_a, points_a[j], axis);
		}

		// Project all points in shape b
		for (unsigned int j = 0; j < count_b; j++)
		{
			projectPoint(limit_b, points_b[j], axis);
		}

		// Overlap found, find solution 
		float delta = limit_b.x - limit_a.y;
		//printf(">axis:(%f, %f), (%f, %f)  (%f, %f), delta=%f\n", axis.x, axis.y, limit_b.x, limit_b.y, limit_a.x, limit_a.y, delta);

		// No overlap found
		if (delta >= 0.0f)
			return false;

		if (delta > overlap.z) 
		{
			overlap = glm::vec3(axis.x, axis.y, delta);
		}
	}

	//printf("=========================== BBB (%f, %f) ===========================\n", pos_b.x, pos_b.y);
	// Check pionts of shape a against axis of shape b
	for (unsigned int i = 0; i < count_b; i++)
	{
		// Get face
		glm::vec4 point_a = points_b[i];
		glm::vec4 point_b = points_b[(i + 1) % count_b];
		// Get projection axis
		glm::vec2 axis = getAxis(point_a, point_b);
		axis = glm::vec2(-axis.y, axis.x);
		glm::vec2 limit_a = glm::vec2(BIG_NUM, -BIG_NUM);
		glm::vec2 limit_b = glm::vec2(BIG_NUM, -BIG_NUM);

		// Project all points in shape a
		for (unsigned int j = 0; j < count_a; j++)
			projectPoint(limit_a, points_a[j], axis);
		// Project all points in shape b
		for (unsigned int j = 0; j < count_b; j++)
			projectPoint(limit_b, points_b[j], axis);

		// Overlap found, find solution 
		float delta = limit_b.x - limit_a.y;
		//printf(">axis:(%f, %f), (%f, %f)  (%f, %f), delta=%f\n", axis.x, axis.y, limit_b.x, limit_b.y, limit_a.x, limit_a.y, delta);

		// No overlap found
		if (delta >= 0.0f)
			return false;

		if (delta > overlap.z) 
		{
			overlap = glm::vec3(axis.x, axis.y, delta);
		}
	}

	//4个顶点在表面法线上的投影
	float offsetY = shape_b->GetHalfExtents().y / 2;
	glm::vec3 normal = glm::vec3(-overlap.x, -overlap.y, 0);
	std::vector<DeepthInfo> tmpList;
	tmpList.push_back(DeepthInfo(0, glm::dot((glm::vec3)points_a[0], normal) - offsetY));
	tmpList.push_back(DeepthInfo(1, glm::dot((glm::vec3)points_a[1], normal) - offsetY));
	tmpList.push_back(DeepthInfo(2, glm::dot((glm::vec3)points_a[2], normal) - offsetY));
	tmpList.push_back(DeepthInfo(3, glm::dot((glm::vec3)points_a[3], normal) - offsetY));
	
	std::sort(tmpList.begin(), tmpList.end(), SortByProjection);
	//printf("point_%d: %f\n", tmpList[0].index, tmpList[0].projection);
	//printf("point_%d: %f\n", tmpList[1].index, tmpList[1].projection);
	//printf("point_%d: %f\n", tmpList[2].index, tmpList[2].projection);
	//printf("point_%d: %f\n", tmpList[3].index, tmpList[3].projection);

	DebugerManager::DrawVector3(body_b->GetPosition(), body_b->GetPosition() + normal);

	//找出投影最小的顶点(最多两个)
	unsigned int MAX_CONCACT_POINTS = 2;
	unsigned int curi = 0;
	float first_projection = 0;
	glm::vec3 contactPoint[2];
	for (int i = 0; i < 4; i++) 
	{
		if (curi >= MAX_CONCACT_POINTS)
			break;

		unsigned int index = tmpList[i].index;
		float projection = tmpList[i].projection;
		if (projection < 0) {
			if (curi == 0) {
				// first point
				contactPoint[curi++] = points_a[index];
				first_projection = projection;
			}
			else {
				// second point
				if (abs(projection - first_projection) <= 0.02f) {
					contactPoint[curi++] = points_a[index];
				}
				else {
					break;
				}
			}
		}

	}

	for(int i = 0; i < curi; i++)
	{
		Contact* contact = cData->contacts;
		contact->penetration = -overlap.z;
		contact->contactNormal = normal;
		contact->contactPoint = contactPoint[i];
		contact->SetBodyList(body_a, body_b);

		contact->restitution = cData->restitution;
		cData->addContacts(1);

		//printf("=== contactPoint: (%f, %f)\n", contact->contactPoint.x, contact->contactPoint.y);
		DebugerManager::DrawPoint(contact->contactPoint);
	}
	
	return true;
}

