#pragma once

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "MyShader.h"
#include "MyGeometry.h"
#include "MyMotionState.h"
#include "Rigidbody2D.h"

typedef struct DrawBatchContext
{
	GLuint vao;
	GLuint vbo;
	GLuint ebo;
	glm::mat4 model;
	glm::mat4 view;
	glm::mat4 projection;
	GLuint mode;
	GLuint count1;
	GLuint count2;
	glm::vec4 color;
}DrawBatchContext;

class GraphicsManager 
{
public:
	GraphicsManager(){}

	static void Init();
	static void ClearBuffers(){m_DrawBatchContext.clear();}
	static void AddDrawBatch(DrawBatchContext dbc);
	static void Render(MyShader* ourShader, glm::mat4 view, glm::mat4 projection);
private:
	static vector<DrawBatchContext> m_DrawBatchContext;
};