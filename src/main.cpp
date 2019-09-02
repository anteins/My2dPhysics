#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "iostream";
#include <thread>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MyCamera.h"
#include "MyShader.h"
#include "MyVector.h"
#include "MyWorld.h"
#include "DebugerManager.h"

#pragma comment(lib, "glfw3.lib")

using namespace My;
using namespace std;

float vertices[] = {
	//---- λ�� ----
	2.0f, 2.0f, 0.0f,		// ���Ͻ�
	2.0f, -0.5f, 0.0f,		// ���½�
	-2.0f, -0.5f, 0.0f,		// ���½�
	-2.0f, 2.0f, 0.0f,		// ���Ͻ�
};

unsigned int indices[] = {
	0, 1, 3, // first triangle
	1, 2, 3  // second triangle
};

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//-----------------------------------
// Common Var
//-----------------------------------
//unsigned int VBO, VAO, EBO;
MyShader* ourShader;

//-----------------------------------
// Input Var
//-----------------------------------
// camera
MyCamera camera(glm::vec3(0.0f, 0.0f, 25.0f));
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = 0.0f;
float lastX = SCR_WIDTH / 2.0;
float lastY = SCR_HEIGHT / 2.0;
float fov = 45.0f;
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;
void processInput(GLFWwindow* window);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

//-----------------------------------
// Physics Var
//-----------------------------------
glm::vec3 G = glm::vec3(0.0f, -9.8f, 0.0f);
MyWorld* world = new MyWorld(G);
#define random(x) (rand()%x)
#define random(a,b) (((double)rand()/RAND_MAX)*(b-a)+a)

void InitShader() 
{
	ourShader = new MyShader("../resource/shader.vs", "../resource/shader.fs");
	ourShader->use();
}

void InitWorld() 
{
	DebugerManager::Init();
	
	Rigidbody2D* body1 = world->CreateBox(glm::vec3(1.0f, 1.0f, 0.0f));
	//Rigidbody2D* body1 = world->CreateBox(glm::vec3(0.0f, 5.0f, 0.0f), glm::vec3(3.0f, 0.5f, 0.0f));
	body1->SetName("body1");
	body1->SetMass(1.0f);
	body1->SetKinematic(false);
	body1->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));

	Rigidbody2D* body2 = world->CreateBox(glm::vec3(3.0f, 0.5f, 0.0f));
	body2->SetName("body2");
	body2->SetMass(1.0f);
	body2->SetKinematic(true);
	body2->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	/*{
		auto collison = body2->GetCollisionShape();
		collison->UpdateRotate(glm::vec3(0, 0, 45));
	}*/

	//DebugerManager::DrawLine(body1->GetCollisionShape()->transform()->position(), body2->GetCollisionShape()->transform()->position(), glm::vec3(0.5,0.5,0.5));
	//DebugerManager::DrawPoint(body1->GetCollisionShape()->GetTransform()->GetLocalPosition(), glm::vec3(0.6, 0.2, 0.9));
}

float auglar = 0.0f;
void UpdateLogic() 
{
	if (world)
	{
		//DebugerManager::ClearDebugBuffers();
		for (auto body : world->bodyList())
		{
			auto collision = body->GetCollisionShape();
			//�޸���ɫ����ʶ�Ƿ���ײ��
			if (body->isColliding)
			{
				collision->SetColor(glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
			}
			else
			{
				collision->SetColor(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
			}

			if (body->GetName() == "body1") 
			{
				//DebugerManager::DrawPoint(body->GetCollisionShape()->transform()->position(), DebugerManager::Color_Red);
				//DebugerManager::DrawBound(body->GetCollisionShape(), DebugerManager::Color_Red);
			}
		}
		auglar = auglar + 1.0f;
	}
}

void Render()
{
	if (world) 
	{
		//model
		glm::mat4 model = glm::mat4(1.0);
		//view
		glm::mat4 view = camera.GetViewMatrix();
		//projection
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		ourShader->use();

		for (auto body : world->bodyList()) 
		{
			std::shared_ptr<MyGeometry> collison = body->GetCollisionShape();
			collison->Draw(ourShader, view, projection);
		}

		//Render Debug Info
		DebugerManager::Draw(ourShader, view, projection);
	}
}

bool pause = false;
void PhysicsThread()
{
	float last = 0.0f;
	while (true) 
	{
		float current = glfwGetTime();
		float pass = current - last;
		if (pass > deltaTime)
		{
			if (!pause) 
			{
				world->Step(deltaTime);
			}
			
			last = current;
		}
	}
}

int main(void)
{
	GLFWwindow* window;

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "ShareTutorial", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	////Set Input CallBack
	//glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	//glfwSetCursorPosCallback(window, mouse_callback);
	//glfwSetScrollCallback(window, scroll_callback);

	//// tell GLFW to capture our mouse
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		//std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	InitShader();
	InitWorld();

	//thread t1(PhysicsThread);
	float last = 0.0f;

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window))
	{
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		processInput(window);

		/* Physics here */
		float current = glfwGetTime();
		float pass = current - last;
		if (pass > deltaTime)
		{
			if (!pause)
			{
				world->Step(deltaTime);
			}

			last = current;
		}

		/* Render here */
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		UpdateLogic();

		Render();

		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();
	}

	/*glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);*/

	glfwTerminate();
	return 0;
}

void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
	{
		int i = 0;
		for (auto body : world->bodyList())
		{
			body->ApplyImpulse(glm::vec3(0.0f, random(0, 7), 0.0f));
			//body->ApplyTorque(glm::vec3(0,0,3));
			i++;
		}
	}
	else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
		pause = !pause;
	}
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}
