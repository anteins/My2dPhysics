#pragma once
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>

#include "MyGeometry.h"
#include "MyQuaternion.h"
#include "MyMotionState.h"
#include "MyBox.h"

using namespace std;

namespace My 
{
	class Rigidbody2D
	{
	public:
		Rigidbody2D(std::shared_ptr<MyGeometry> shape) : m_pCollisionShape(shape)
		{
			this->m_id = 0;
			this->m_gravity = glm::vec3(0, -9.8f, 0);
			this->linearDamping = 0.8f;
			this->angularDamping = 0.8f;
			this->isColliding = false;
			this->velocity = glm::vec3(0.0f);
			this->angularAcceleration = glm::vec3(0.0f);
			this->angularVelocity = glm::vec3(0.0f);
			this->torque = glm::vec3(0.0f);

			this->centerPos = glm::vec3(0.0f);
			this->m_motionState = new MyMotionState();

			this->m_pCollisionShape->InitShape(this->centerPos, this->m_motionState);
		};

		void SetPosition(glm::vec3 position) 
		{ 
			this->m_motionState->Translate(position);
			this->m_pCollisionShape->UpdateBound();
		}

		glm::vec3 GetPosition() 
		{
			return this->m_motionState->GetWorldPos();
		}

		void UpdatePosition(glm::vec3 posDelta)
		{
			this->m_motionState->Translate(posDelta);
			this->m_pCollisionShape->UpdateBound();
		}

		void SetRotate(float angular)
		{
			this->m_motionState->Rotate(angular);
			this->m_pCollisionShape->UpdateBound();
		}

		void UpdateRotate(float angular)
		{
			this->angular += angular;
			this->m_motionState->Rotate(angular);
			this->m_pCollisionShape->UpdateBound();
		}

		AabbBound GetAabbBound()
		{
			std::shared_ptr<MyBox> boxShape = std::static_pointer_cast<MyBox>(this->m_pCollisionShape);
			if (boxShape) {
				return boxShape->GetBound();
			}
		}

		MyMotionState* transform() { return this->m_motionState; }

		void SetVelocity(glm::vec3 velocity) { this->velocity = velocity; }

		void SetId(unsigned int id) {this->m_id = id;}

		const unsigned int GetId() { return this->m_id; }

		std::shared_ptr<MyGeometry> GetShape() { return m_pCollisionShape; }

		void Render(MyShader* ourShader, glm::mat4& view, glm::mat4& projection)
		{
			this->m_pCollisionShape->Render(ourShader, this->transform()->GetMatrix(), view, projection);
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

		void SetOrientation(const Quaternion &orientation);
		void SetOrientation(const double r, const double i,
			const double j, const double k);
		void GetOrientation(Quaternion *orientation) const;

		MyMotionState* m_motionState;

		float mass;
		float inv_mass;

		float linearDamping;
		float angularDamping;

		float restitution;
		bool isKinematic;
		bool isColliding;

		glm::vec3 centerPos;

		glm::vec3 force;
		glm::vec3 acceleration;
		glm::vec3 velocity;

		glm::vec3 torque;
		glm::vec3 angularAcceleration;
		glm::vec3 angularVelocity;
		float angular;

		Quaternion orientation;
		glm::vec3 rotation;

	private:
		unsigned int m_id;
		std::shared_ptr<MyGeometry> m_pCollisionShape;
		glm::vec3 m_gravity;
		~Rigidbody2D() {};
	};
}