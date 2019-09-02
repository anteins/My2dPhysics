#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace My {
	inline void TransformAabb(const glm::vec3& halfExtents, float margin, const glm::mat4& trans,
		glm::vec3& aabbMinOut, glm::vec3& aabbMaxOut)
	{
		glm::vec3 halfExtentsWithMargin = halfExtents + glm::vec3(margin);
		glm::vec3  center;
		glm::vec3  extent;
		glm::mat3 basis;
		/*GetOrigin(center, trans);
		Shrink(basis, trans);
		Absolute(basis, basis);
		DotProduct3(extent, halfExtentsWithMargin, basis);
		aabbMinOut = center - extent;
		aabbMaxOut = center + extent;*/
	}
}