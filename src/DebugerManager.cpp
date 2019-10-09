#include "DebugerManager.h"
#include "MyGeometry.h"
#include <glm/gtx/string_cast.hpp>

//Debug Color Enum
glm::vec3 DebugerManager::Color_White = glm::vec3(1.0f);
glm::vec3 DebugerManager::Color_Red = glm::vec3(1.0, 0.0, 0.0);
glm::vec3 DebugerManager::Color_Green = glm::vec3(0.0, 1.0, 0.0);
glm::vec3 DebugerManager::Color_Blue = glm::vec3(0.0, 0.0, 1.0);
glm::vec3 DebugerManager::Color_Yellow = glm::vec3(1.0, 1.0, 0.0);

vector<DebugDrawBatchContext> DebugerManager::m_DebugDrawBatchContext = vector<DebugDrawBatchContext>();
vector<DebugDrawBatchContext> DebugerManager::m_NoClearDebugDrawBatchContext = vector<DebugDrawBatchContext>();

void DebugerManager::DrawBound(Rigidbody2D* body, const glm::vec3& color)
{
	AabbBound bound = body->GetAabbBound();
	glm::mat4 transform = body->GetMatrix();

	/*GLfloat vertices[6];
	vertices[0] = bound.min.x;
	vertices[1] = bound.min.y;
	vertices[2] = bound.min.z;

	vertices[3] = bound.max.x;
	vertices[4] = bound.max.y;
	vertices[5] = bound.max.z;
*/
	glm::vec3 vertices[2];
	vertices[0] = bound.min;
	vertices[1] = bound.max;

	GLuint vao;
	glGenVertexArrays(1, &vao);

	// Bind the vertex array object to store all the buffers and vertex attributes we create here.
	glBindVertexArray(vao);

	GLuint buffer_id;

	// Generate an ID for the vertex buffer.
	glGenBuffers(1, &buffer_id);

	// Bind the vertex buffer and load the vertex (position and color) data into the vertex buffer.
	glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);

	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

	DebugDrawBatchContext& dbc = *(new DebugDrawBatchContext);
	dbc.vao = vao;
	dbc.mode = GL_POINTS;
	dbc.count = 2;
	dbc.pointSize = 3;
	dbc.color = color;

	m_DebugDrawBatchContext.push_back(std::move(dbc));
}

void DebugerManager::DrawPoint(glm::vec3 position, const glm::vec3& color)
{
	GLfloat vertices[3];
	vertices[0] = position.x;
	vertices[1] = position.y;
	vertices[2] = position.z;

	GLuint vao;
	glGenVertexArrays(1, &vao);

	// Bind the vertex array object to store all the buffers and vertex attributes we create here.
	glBindVertexArray(vao);

	GLuint buffer_id;

	// Generate an ID for the vertex buffer.
	glGenBuffers(1, &buffer_id);

	// Bind the vertex buffer and load the vertex (position and color) data into the vertex buffer.
	glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);

	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

	DebugDrawBatchContext& dbc = *(new DebugDrawBatchContext);
	dbc.vao = vao;
	dbc.mode = GL_POINTS;
	dbc.count = 1;
	dbc.pointSize = 3;
	dbc.color = color;

	m_NoClearDebugDrawBatchContext.push_back(std::move(dbc));
}

void DebugerManager::DrawLine(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color)
{
	GLfloat vertices[6];
	vertices[0] = from.x;
	vertices[1] = from.y;
	vertices[2] = from.z;

	vertices[3] = to.x;
	vertices[4] = to.y;
	vertices[5] = to.z;

	GLuint vao;
	glGenVertexArrays(1, &vao);

	// Bind the vertex array object to store all the buffers and vertex attributes we create here.
	glBindVertexArray(vao);

	GLuint buffer_id;

	// Generate an ID for the vertex buffer.
	glGenBuffers(1, &buffer_id);

	// Bind the vertex buffer and load the vertex (position and color) data into the vertex buffer.
	glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);

	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

	DebugDrawBatchContext& dbc = *(new DebugDrawBatchContext);
	dbc.vao = vao;
	dbc.mode = GL_LINES;
	dbc.count = 2;
	dbc.color = color;

	m_NoClearDebugDrawBatchContext.push_back(std::move(dbc));
}

void DebugerManager::DrawVector3(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color)
{
	DrawLine(from, to, color);
}

void DebugerManager::Draw(MyShader* ourShader, glm::mat4 view, glm::mat4 projection)
{
	// Set the color shader as the current shader program and set the matrices that it will use for rendering.
	ourShader->use();
	ourShader->setMat4("model", glm::mat4(1.0f));
	ourShader->setMat4("view", view);
	ourShader->setMat4("projection", projection);

	for (auto dbc : m_DebugDrawBatchContext)
	{
		if (dbc.pointSize > 0)
		{
			glPointSize(dbc.pointSize);
		}

		/*PrintVec3("dbc.color: ", dbc.color);*/
		ourShader->setVec4("vertexColor", glm::vec4(dbc.color, 1.0f));

		glBindVertexArray(dbc.vao);
		glDrawArrays(dbc.mode, 0x00, dbc.count); // 绘制调试信息
	}

	//Not Clear
	for (auto dbc : m_NoClearDebugDrawBatchContext)
	{
		if (dbc.pointSize > 0)
		{
			glPointSize(dbc.pointSize);
		}

		/*PrintVec3("dbc.color: ", dbc.color);*/
		ourShader->setVec4("vertexColor", glm::vec4(dbc.color, 1.0f));

		glBindVertexArray(dbc.vao);
		glDrawArrays(dbc.mode, 0x00, dbc.count); // 绘制调试信息
	}
}

void DebugerManager::PrintVec3(glm::vec3 vector, const std::string& headMessage)
{
	printf("%s(%f, %f, %f)\n", headMessage.c_str(), vector.x, vector.y, vector.z);
}
