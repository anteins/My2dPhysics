#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Rigidbody2D.h"

using namespace My;

class Contact
{
public:
	Rigidbody2D* body[2];
	glm::vec3 contactNormal;
	glm::vec3 contactPoint;

	float penetration;
	float restitution;

	float desiredDeltaVelocity;

	glm::mat3 contactToWorld;
	glm::mat3 worldToContact;

	glm::vec3 relativePosition[2];
	glm::vec3 contactVelocity;

	void SetBodyList(Rigidbody2D* body_a, Rigidbody2D* body_b);
	void CalculateInternals(float dt);
	void CreateContactBasic();
	glm::vec3 CalculateLocalVelocity(unsigned int index, float dt);
	glm::vec3 CalculateFrictionlessImpulse(float dt);
	void ApplyPosition(float dt);
	void ApplyVelocity(float dt);
};

class CollisionData
{
public:
	/**
	 * Holds the base of the collision data: the first contact
	 * in the array. This is used so that the contact pointer (below)
	 * can be incremented each time a contact is detected, while
	 * this pointer points to the first contact found.
	 */
	Contact* contactArray;

	/** Holds the contact array to write into. */
	Contact* contacts;

	/** Holds the maximum number of contacts the array can take. */
	int contactsLeft;

	/** Holds the number of contacts found so far. */
	unsigned contactCount;

	/** Holds the friction value to write into any collisions. */
	float friction;

	/** Holds the restitution value to write into any collisions. */
	float restitution;

	/**
	 * Holds the collision tolerance, even uncolliding objects this
	 * close should have collisions generated.
	 */
	float tolerance;

	/**
	 * Checks if there are more contacts available in the contact
	 * data.
	 */
	bool hasMoreContacts()
	{
		return contactsLeft > 0;
	}

	/**
	 * Resets the data so that it has no used contacts recorded.
	 */
	void reset(unsigned maxContacts)
	{
		contactsLeft = maxContacts;
		contactCount = 0;
		contacts = contactArray;
	}

	/**
	 * Notifies the data that the given number of contacts have
	 * been added.
	 */
	void addContacts(unsigned count)
	{
		// Reduce the number of contacts remaining, add number used
		contactsLeft -= count;
		contactCount += count;

		// Move the array forward
		contacts += count;
	}
};

