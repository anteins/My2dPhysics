#pragma once
#include <memory>
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
			this->velocity = glm::vec3(0.0f);
			this->angularAcceleration = glm::vec3(0.0f);
			this->rotation = glm::vec3(0.0f);
			this->torque = glm::vec3(0.0f);
			this->m_motionState = new MyMotionState();
			this->m_pCollisionShape->InitShape(this->m_motionState);
			this->isColliding = false;
			this->inv_mass = 0;
		};

		void SetId(unsigned int id) { this->m_id = id; }

		const unsigned int GetId() { return this->m_id; }

		void SetVelocity(glm::vec3 velocity) { this->velocity = velocity; }

		void CalculateDerivedData();

		void SetPosition(glm::vec3 position) 
		{ 
			this->m_motionState->Translate(position);
			this->m_pCollisionShape->UpdateBound();
		}

		glm::vec3 GetVelocity() { return this->velocity; }
		MyMotionState* Transform() { return this->m_motionState; }
		glm::vec3 GetPosition() { return this->m_motionState->GetAxis(3); }
		glm::vec3 GetRotate() { return this->rotation; }
		glm::vec3 GetAxis(unsigned int index){ return this->m_motionState->GetAxis(index); }
		glm::mat4 GetMat44() { return this->Transform()->GetMat44(); }
		std::shared_ptr<MyGeometry> GetShape() { return m_pCollisionShape; }

		void SetRotate(glm::vec3 rotation)
		{
			this->anglar = rotation;
			this->m_motionState->Rotate(this->anglar.z);
			this->m_pCollisionShape->UpdateBound();
		}

		void AddRotate(glm::vec3 rotation)
		{
			this->anglar = this->anglar + rotation;
			this->m_motionState->Rotate(this->anglar.z);
			this->m_pCollisionShape->UpdateBound();
		}

		AabbBound GetAabbBound()
		{
			AabbBound bound;
			std::shared_ptr<MyBox> boxShape = std::static_pointer_cast<MyBox>(this->m_pCollisionShape);
			if (boxShape) 
			{
				bound = boxShape->GetBound();
			}
			return bound;
		}

		void Render(MyShader* ourShader, glm::mat4& view, glm::mat4& projection)
		{
			this->m_pCollisionShape->Render(ourShader, this->GetMat44(), view, projection);
		}

		void SetColor(glm::vec4 color) { this->m_pCollisionShape->SetColor(color); }

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
		void AddTorque(glm::vec3 torque);
		void ApplyImpulse(glm::vec3 impulse);
		void ApplyTorqueImpulse(glm::vec3 impulse);

		void CalcAllForce();
		void Integrate(float dt);

		void SetOrientation(const MyQuaternion &orientation);
		void SetOrientation(const double r, const double i,
			const double j, const double k);
		void GetOrientation(MyQuaternion *orientation) const;

		glm::mat3 inverseInertiaTensorWorld;
		glm::mat3 inverseInertiaTensor;

		MyMotionState* m_motionState;

		float mass;
		float inv_mass;

		float linearDamping;
		float angularDamping;

		bool isKinematic;
		bool isColliding;

		glm::vec3 force;
		glm::vec3 acceleration;
		glm::vec3 velocity;

		glm::vec3 torque;
		glm::vec3 angularAcceleration;
		glm::vec3 anglar;
		glm::vec3 rotation;

		MyQuaternion orientation;

	private:
		unsigned int m_id;
		std::shared_ptr<MyGeometry> m_pCollisionShape;
		glm::vec3 m_gravity;
		~Rigidbody2D() {};
	};
}