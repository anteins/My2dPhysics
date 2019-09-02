#pragma once

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "MyShader.h"
#include "MyGeometry.h"
#include "MyMotionState.h"

using namespace std;

typedef struct DebugDrawBatchContext 
{
	GLuint vao;
	GLuint mode;
	GLuint count;
	GLuint pointSize;
	glm::vec3 color;
}DebugDrawBatchContext;

class DebugerManager 
{
public:
	static void Init()
	{
		m_debugShaderProgram = new MyShader("resource/shader.vs", "resource/shader.fs");
	}

	static void DrawPoint(glm::vec3 position, unsigned int size = 5, const glm::vec3& color = Color_White);
	static void DrawLine(const glm::vec3& to, const glm::vec3& from, const glm::vec3& color = Color_White);
	static void DrawVector3(const glm::vec3& to, const glm::vec3& from, const glm::vec3& color = Color_White);
	static void DrawBound(std::shared_ptr<MyGeometry> collisionShape, const glm::vec3& color = Color_White);
	static void ClearDebugBuffers()
	{
		m_DebugDrawBatchContext.clear();
	}
	static void AddBuffer(unsigned int bufferId) { m_Buffers.push_back(bufferId); }
	static void Draw(MyShader* ourShader, glm::mat4 view, glm::mat4 projection);

	static void PrintVec3(const std::string& name, glm::vec3 vector);

	static glm::vec3 Color_Red;
	static glm::vec3 Color_White;
private:
	static vector<GLuint> m_Buffers;
	static MyShader* m_debugShaderProgram;
	static vector<DebugDrawBatchContext> m_DebugDrawBatchContext;
};