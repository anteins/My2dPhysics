#include "GraphicsManager.h"
#include "MyGeometry.h"

vector<DrawBatchContext> GraphicsManager::m_DrawBatchContext = vector<DrawBatchContext>();

void GraphicsManager::Init() 
{
}

void GraphicsManager::AddDrawBatch(DrawBatchContext dbc)
{
	m_DrawBatchContext.push_back(std::move(dbc));
}

void GraphicsManager::Render(MyShader* ourShader, glm::mat4 view, glm::mat4 projection)
{
	// Set the color shader as the current shader program and set the matrices that it will use for rendering.
	ourShader->use();
	for (auto dbc : m_DrawBatchContext)
	{
		ourShader->setMat4("model", dbc.model);
		ourShader->setMat4("view", dbc.view);
		ourShader->setMat4("projection", dbc.projection);
		ourShader->setVec4("vertexColor", dbc.color);

		glBindVertexArray(dbc.vao);

		switch (dbc.mode) 
		{
			case GL_TRIANGLES:
				//glDrawArrays(dbc.mode, 0x00, dbc.count);
				glDrawElements(dbc.mode, dbc.count1, GL_UNSIGNED_INT, 0);
				break;
			case GL_LINE_LOOP:
				glEnableVertexAttribArray(0);
				glBindBuffer(GL_ARRAY_BUFFER, dbc.vbo);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
				glDrawArrays(GL_LINE_LOOP, dbc.count2, dbc.count2);
				break;
		}
	}
}

