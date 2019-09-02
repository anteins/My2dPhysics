#pragma once

#include <vector>

#include "MyVector.h"
#include "Rigidbody2D.h"
#include "ContactData.h"

namespace My 
{
	class MyWorld 
	{
	public:
		MyWorld(glm::vec3& gravity):gravity(gravity){};

		Rigidbody2D* CreateBox(glm::vec3 halfExtents);
		Rigidbody2D* CreateSphere(glm::vec3 center, float r);
		void Step(float dt);
		bool Collide(Rigidbody2D* body1, Rigidbody2D* body2, ContactData* data, float dt);
		const std::vector<Rigidbody2D*>& bodyList() const{return m_bodyList;}
	private:
		glm::vec3 gravity;
		std::vector<Rigidbody2D*> m_bodyList;
	};
}
