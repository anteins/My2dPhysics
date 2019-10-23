#pragma once

#include <vector>
#include "MyVector.h"
#include "Rigidbody2D.h"
#include "ContactData.h"

#define maxContacts 256

namespace My 
{
	class MyWorld 
	{
	public:
		MyWorld(glm::vec3& gravity) :gravity(gravity) { this->cData.contactArray = this->contacts;  };

		Rigidbody2D* CreateBox(glm::vec3 halfExtents);
		Rigidbody2D* CreateSphere(float r);
		void Step(float dt);
		bool Collide(Rigidbody2D* body1, Rigidbody2D* body2, CollisionData& cData, float dt);
		const std::vector<Rigidbody2D*>& bodyList() const{return m_bodyList;}
	private:
		glm::vec3 gravity;
		std::vector<Rigidbody2D*> m_bodyList;
		Contact contacts[maxContacts];
		CollisionData cData;
	};
}
