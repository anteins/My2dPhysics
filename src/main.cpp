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
#include "GraphicsManager.h"

#pragma comment(lib, "glfw3.lib")

using namespace My;
using namespace std;

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
MyCamera camera(glm::vec3(0.0f, 0.0f, 18.0f));
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

void Demo1() 
{
	/*Rigidbody2D* box1 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box1->SetId(1);
	box1->SetPosition(glm::vec3(-5.0f, 2.3f, 0.0f));
	box1->SetRotate(glm::vec3(0, 0, 0));
	box1->SetMass(16.0f);
	box1->SetKinematic(false);*/

	Rigidbody2D* sphere1 = world->CreateSphere(0.3);
	sphere1->SetId(1);
	sphere1->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
	sphere1->SetMass(16.0f);
	sphere1->SetKinematic(false);

	Rigidbody2D* panel4 = world->CreateBox(glm::vec3(15.0f, 0.5f, 0.0f));
	panel4->SetId(6);
	panel4->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	panel4->SetRotate(glm::vec3(0, 0, -9));
	panel4->SetMass(16.0f);
	panel4->SetKinematic(true);
}

void Demo3()
{
	unsigned int serId = 0;
	Rigidbody2D* box1 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box1->SetId(serId++);
	box1->SetPosition(glm::vec3(0.0f, 2.0f, 0.0f));
	box1->SetRotate(glm::vec3(0, 0, 0));
	box1->SetMass(16.0f);
	box1->SetKinematic(false);

	Rigidbody2D* box2 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box2->SetId(serId++);
	box2->SetPosition(glm::vec3(0.0f, 5.5f, 0.0f));
	box2->SetRotate(glm::vec3(0, 0, -30));
	box2->SetMass(16.0f);
	box2->SetKinematic(false);

	Rigidbody2D* box_3 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box_3->SetId(serId++);
	box_3->SetPosition(glm::vec3(2.5f, 7.5f, 0.0f));
	box_3->SetRotate(glm::vec3(0, 0, -30));
	box_3->SetMass(1.0f);
	box_3->SetKinematic(false);

	Rigidbody2D* box_4 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box_4->SetId(serId++);
	box_4->SetPosition(glm::vec3(4.0f, 8.5f, 0.0f));
	box_4->SetRotate(glm::vec3(0, 0, -10));
	box_4->SetMass(3.0f);
	box_4->SetKinematic(false);

	Rigidbody2D* panel2 = world->CreateBox(glm::vec3(4.0f, 0.3f, 0.0f));
	panel2->SetId(serId++);
	panel2->SetPosition(glm::vec3(0.0f, 1.5f, 0.0f));
	panel2->SetRotate(glm::vec3(0, 0, -15));
	panel2->SetMass(16.0f);
	panel2->SetKinematic(true);

	Rigidbody2D* wall_1 = world->CreateBox(glm::vec3(1.2f, 0.2f, 0.0f));
	wall_1->SetId(serId++);
	wall_1->SetPosition(glm::vec3(3.0f, 1.0f, 0.0f));
	wall_1->SetRotate(glm::vec3(0, 0, -90));
	wall_1->SetMass(16.0f);
	wall_1->SetKinematic(true);

	Rigidbody2D* panel1 = world->CreateBox(glm::vec3(10.0f, 0.3f, 0.0f));
	panel1->SetId(serId++);
	panel1->SetPosition(glm::vec3(0.0f, -1.0f, 0.0f));
	panel1->SetRotate(glm::vec3(0, 0, 15));
	panel1->SetMass(16.0f);
	panel1->SetKinematic(true);


	Rigidbody2D* panel3 = world->CreateBox(glm::vec3(5.0f, 0.3f, 0.0f));
	panel3->SetId(serId++);
	panel3->SetPosition(glm::vec3(-6.0f, -4.0f, 0.0f));
	panel3->SetRotate(glm::vec3(0, 0, -60));
	panel3->SetMass(16.0f);
	panel3->SetKinematic(true);

	Rigidbody2D* panel_4 = world->CreateBox(glm::vec3(3.0f, 0.3f, 0.0f));
	panel_4->SetId(serId++);
	panel_4->SetPosition(glm::vec3(-3.4f, -5.9f, 0.0f));
	panel_4->SetRotate(glm::vec3(0, 0, 30));
	panel_4->SetMass(16.0f);
	panel_4->SetKinematic(true);
}

void Demo2() 
{
	//Rigidbody2D* sphere1 = world->CreateSphere(0.8);
	//sphere1->SetId(1);
	//sphere1->SetPosition(glm::vec3(0.0f, 7.0f, 0.0f));
	//sphere1->SetMass(1.0f);
	//sphere1->SetKinematic(false);

	/*Rigidbody2D* box1 = world->CreateBox(glm::vec3(1.0f, 1.0f, 0.0f));
	box1->SetId(1);
	box1->SetPosition(glm::vec3(0.0f, 5.0f, 0.0f));
	box1->SetRotate(glm::vec3(0, 0, 45));
	box1->SetMass(16.0f);
	box1->SetKinematic(false);*/

	/*Rigidbody2D* box2 = world->CreateBox(glm::vec3(0.5f, 0.5f, 0.0f));
	box2->SetId(2);
	box2->SetPosition(glm::vec3(1.0f, 1.0f, 0.0f));
	box2->SetMass(1.0f);
	box2->SetKinematic(false);*/

	//Rigidbody2D* panel1 = world->CreateBox(glm::vec3(8.0f, 0.15f, 0.0f));
	//panel1->SetId(3);
	//panel1->SetPosition(glm::vec3(3.0f, 5.0f, 0.0f));
	//panel1->SetRotate(17);
	//panel1->SetMass(1.0f);
	//panel1->SetKinematic(true);

	/*Rigidbody2D* panel2 = world->CreateBox(glm::vec3(8.0f, 0.15f, 0.0f));
	panel2->SetId(4);
	panel2->SetPosition(glm::vec3(-3.0f, 2.0f, 0.0f));
	panel2->SetRotate(-17);
	panel2->SetMass(1.0f);
	panel2->SetKinematic(true);

	rotate_panel = world->CreateBox(glm::vec3(11.0f, 0.15f, 0.0f));
	rotate_panel->SetId(5);
	rotate_panel->SetPosition(glm::vec3(3.0f, -1.0f, 0.0f));
	rotate_panel->SetRotate(35);
	rotate_panel->SetMass(1.0f);
	rotate_panel->SetKinematic(true);*/

	Rigidbody2D* panel4 = world->CreateBox(glm::vec3(10.0f, 0.3f, 0.0f));
	panel4->SetId(6);
	panel4->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	panel4->SetRotate(glm::vec3(0));
	panel4->SetMass(16.0f);
	panel4->SetKinematic(true);
}

void InitWorld() 
{
	GraphicsManager::Init();
	DebugerManager::Init();

	Demo1();
}

void UpdateLogic() 
{
	if (world)
	{
		for (auto body : world->bodyList())
		{
			auto collision = body->GetShape();
			//修改颜色（标识是否碰撞）
			if (body->isColliding)
			{
				collision->SetColor(glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
			}
			else
			{
				collision->SetColor(glm::vec4(1.0f));
			}
		}
	}
}

void Render()
{
	if (world) 
	{
		GraphicsManager::ClearBuffers();
		DebugerManager::ClearDebugBuffers();

		//view
		glm::mat4 view = camera.GetViewMatrix();
		//projection
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		ourShader->use();

		for (auto body : world->bodyList()) 
		{
			body->Render(ourShader, view, projection);
		}

		GraphicsManager::Render(ourShader, view, projection);
		//Render Debug Info
		DebugerManager::Draw(ourShader, view, projection);
	}
}

bool pause = true;
void PhysicsThread()
{
	float last = 0.0f;
	while (true) 
	{
		/*float current = glfwGetTime();
		float pass = current - last;
		if (pass > deltaTime)
		{
			if (!pause) 
			{
				world->Step(deltaTime);
			}
			
			last = current;
		}*/

		if (!pause)
		{
			world->Step(0.016f);
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

		UpdateLogic();

		/* Physics here */
		/*float current = glfwGetTime();
		float pass = current - last;
		if (pass > deltaTime)
		{
			if (!pause)
			{
				world->Step(deltaTime);
			}

			last = current;
		}*/
		if (!pause)
		{
			world->Step(0.016f);
		}

		/* Render here */
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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

float last_keyon = 0;
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
		float current = glfwGetTime();
		if (last_keyon != 0)
		{
			float pass = abs(current - last_keyon);
			//点击间隔
			if (pass < 0.3f)
			{
				return;
			}
		}
		last_keyon = current;

		printf("F!\n");
		
		int i = 0;
		for (auto body : world->bodyList())
		{
			//body->ApplyImpulse(glm::vec3(0.0f, random(0, 7), 0.0f));
			body->ApplyTorqueImpulse(glm::vec3(0, 0, 150));
			i++;
		}
	}
	else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
		float current = glfwGetTime();
		if (last_keyon != 0)
		{
			float pass = abs(current - last_keyon);
			//点击间隔
			if (pass < 0.3f)
			{
				return;
			}
		}
		last_keyon = current;

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
