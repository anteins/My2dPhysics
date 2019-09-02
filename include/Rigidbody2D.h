#pragma once
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "MyGeometry.h"
#include <string>

using namespace std;

namespace My 
{
	class Rigidbody2D
	{
	public:
		Rigidbody2D(std::shared_ptr<MyGeometry> collisionShape) : m_pCollisionShape(collisionShape)
		{
			this->m_name = "";
			this->_gravity = glm::vec3(0, -9.8f, 0);
			this->linearDamping = 1.0f;
			this->angularDamping = 1.0f;
			this->isColliding = false;
			this->velocity = glm::vec3(0.0f);

			this->angularAcceleration = glm::vec3(0.0f);
			this->angularVelocity = glm::vec3(0.0f);
			this->torque = glm::vec3(0.0f);
		};

		std::shared_ptr<MyGeometry> GetCollisionShape() { return m_pCollisionShape; }

		void SetPosition(glm::vec3 position) 
		{
			m_pCollisionShape->UpdatePosition(position);
		}

		void SetName(const string name) 
		{
			this->m_name = name;
		}

		const string GetName() 
		{
			return this->m_name;
		}

		void SetMass(float mass) 
		{ 
			this->mass = mass;
			if (this->mass == 0)
			{
				this->inv_mass = 0;
			}
			else
			{
				this->inv_mass = 1 / this->mass;
			}
		}
		void SetKinematic(bool isKinematic) { this->isKinematic = isKinematic; }
		bool IsKinematic() {return isKinematic;}
		void AddForce(glm::vec3 force);
		void ApplyTorque(glm::vec3 impulse);
		void ApplyImpulse(glm::vec3 impulse);
		void CalcAllForce();
		void Integrate(float dt);

		float linearDamping;
		float angularDamping;
		float mass;
		float inv_mass;
		float restitution;
		bool isKinematic;
		bool isColliding;

		glm::vec3 force;
		glm::vec3 velocity;
		glm::vec3 acceleration;

		glm::vec3 angularAcceleration;
		glm::vec3 angularVelocity;
		glm::vec3 torque;

	private:
		string m_name;

		std::shared_ptr<MyGeometry> m_pCollisionShape;
		glm::vec3 _gravity;
		~Rigidbody2D() {};
	};
}