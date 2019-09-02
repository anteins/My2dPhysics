#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec4 VertexColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec4 vertexColor;

void main()
{
	vec4 pos2 = model * vec4(aPos, 1.0);
	gl_Position = projection * view * pos2;
	VertexColor = vertexColor;
}