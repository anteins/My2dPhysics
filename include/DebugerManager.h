#pragma once

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "MyShader.h"
#include "MyGeometry.h"
#include "MyMotionState.h"
#include "Rigidbody2D.h"

using namespace std;
using namespace My;

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
		//m_debugShaderProgram = new MyShader("../resource/shader.vs", "../resource/shader.fs");
	}

	static void DrawPoint(glm::vec3 position, const glm::vec3& color = Color_Red);
	static void DrawLine(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color = Color_White);
	static void DrawVector3(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color = Color_White);

	static void DrawBound(Rigidbody2D* body, const glm::vec3& color = Color_White);

	static void ClearDebugBuffers()
	{
		m_DebugDrawBatchContext.clear();
	}
	static void Draw(MyShader* ourShader, glm::mat4 view, glm::mat4 projection);

	static void PrintVec3(glm::vec3 vector, const std::string& headMessage = "");

	static glm::vec3 Color_White;
	static glm::vec3 Color_Red;
	static glm::vec3 Color_Green;
	static glm::vec3 Color_Blue;
	static glm::vec3 Color_Yellow;

private:
	//static MyShader* m_debugShaderProgram;
	static vector<DebugDrawBatchContext> m_DebugDrawBatchContext;
	static vector<DebugDrawBatchContext> m_NoClearDebugDrawBatchContext;
};