/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#define GLFW_INCLUDE_NONE
#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "our-physics/OurWorld.h"
#include "our-physics/OurBody.h"
#include "our-physics/OurJoint.h"

namespace
{
	GLFWwindow* mainWindow = NULL;

	OurBody bodies[200];
	OurJoint joints[100];
	
	OurBody* bomb = NULL;

	float timeStep = 1.0f / 60.0f;
	int iterations = 10;
	V2 gravity(0.0f, -10.0f);

	int numBodies = 0;
	int numJoints = 0;

	int demoIndex = 0;

	int width = 1280;
	int height = 720;
	float zoom = 10.0f;
	float pan_y = 8.0f;

	OurWorld world(gravity, iterations);
	
	//상태 저장 변수 추가
	// (일시정지)
	bool flag = false;
}

static void glfwErrorCallback(int error, const char* description)
{
	printf("GLFW error %d: %s\n", error, description);
}

static void DrawText(int x, int y, const char* string)
{
	ImVec2 p;
	p.x = float(x);
	p.y = float(y);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(p);
	ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
	ImGui::End();
}

static void DrawBody(OurBody* body)
{
	M22 R(body->rotation);
	V2 x = body->position;
	V2 h = 0.5f * body->width;

	V2 v1 = x + R * V2(-h.x, -h.y);
	V2 v2 = x + R * V2( h.x, -h.y);
	V2 v3 = x + R * V2( h.x,  h.y);
	V2 v4 = x + R * V2(-h.x,  h.y);

	if (body == bomb)
		glColor3f(0.4f, 0.9f, 0.4f);
	else
		glColor3f(0.8f, 0.8f, 0.9f);

	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

static void DrawJoint(OurJoint* joint)
{
	Body* b1 = joint->body1;
	Body* b2 = joint->body2;

	M22 R1(b1->rotation);
	M22 R2(b2->rotation);

	V2 x1 = b1->position;
	V2 p1 = x1 + R1 * joint->localAnchor1;

	V2 x2 = b2->position;
	V2 p2 = x2 + R2 * joint->localAnchor2;

	glColor3f(0.5f, 0.5f, 0.8f);
	glBegin(GL_LINES);
	glVertex2f(x1.x, x1.y);
	glVertex2f(p1.x, p1.y);
	glVertex2f(x2.x, x2.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
}

static void LaunchBomb()
{
	if (!bomb)
	{
		bomb = bodies + numBodies;
		bomb->Set(V2(1.0f, 1.0f), 50.0f);
		bomb->friction = 0.2f;
		world.Add(bomb);
		++numBodies;
	}

	bomb->position.Set(Random(-15.0f, 15.0f), 15.0f);
	bomb->rotation = Random(-1.5f, 1.5f);
	bomb->velocity = -1.5f * bomb->position;
	bomb->angularVelocity = Random(-20.0f, 20.0f);
}

// Single box
static void Demo1(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 4.0f);
	world.Add(b);
	++b; ++numBodies;
}

// A simple pendulum
static void Demo2(OurBody* b, OurJoint* j)
{
	OurBody* b1 = b + 0;
	b1->Set(V2(100.0f, 20.0f), FLT_MAX);
	b1->friction = 0.2f;
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	b1->rotation = 0.0f;
	world.Add(b1);

	OurBody* b2 = b + 1;
	b2->Set(V2(1.0f, 1.0f), 100.0f);
	b2->friction = 0.2f;
	b2->position.Set(9.0f, 11.0f);
	b2->rotation = 0.0f;
	world.Add(b2);

	numBodies += 2;

	j->Set(b1, b2, V2(0.0f, 11.0f));
	world.Add(j);

	numJoints += 1;
}

// Varying friction coefficients
static void Demo3(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 11.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(5.25f, 9.5f);
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(2.0f, 7.0f);
	b->rotation = 0.25f;
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(-5.25f, 5.5f);
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 3.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; ++numBodies;

	float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
	for (int i = 0; i < 5; ++i)
	{
		b->Set(V2(0.5f, 0.5f), 25.0f);
		b->friction = friction[i];
		b->position.Set(-7.5f + 2.0f * i, 14.0f);
		world.Add(b);
		++b; ++numBodies;
	}
}

// A vertical stack
static void Demo4(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(V2(1.0f, 1.0f), 1.0f);
		b->friction = 0.2f;
		float x = Random(-0.1f, 0.1f);
		b->position.Set(x, 0.51f + 1.05f * i);
		world.Add(b);
		++b; ++numBodies;
	}
}

// A pyramid
static void Demo5(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	V2 x(-6.0f, 0.75f);
	V2 y;

	for (int i = 0; i < 12; ++i)
	{
		y = x;

		for (int j = i; j < 12; ++j)
		{
			b->Set(V2(1.0f, 1.0f), 10.0f);
			b->friction = 0.2f;
			b->position = y;
			world.Add(b);
			++b; ++numBodies;

			y += V2(1.125f, 0.0f);
		}

		//x += V2(0.5625f, 1.125f);
		x += V2(0.5625f, 2.0f);
	}
}

// A teeter
static void Demo6(OurBody* b, OurJoint* j)
{
	OurBody* b1 = b + 0;
	b1->Set(V2(100.0f, 20.0f), FLT_MAX);
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	world.Add(b1);

	OurBody* b2 = b + 1;
	b2->Set(V2(12.0f, 0.25f), 100.0f);
	b2->position.Set(0.0f, 1.0f);
	world.Add(b2);

	OurBody* b3 = b + 2;
	b3->Set(V2(0.5f, 0.5f), 25.0f);
	b3->position.Set(-5.0f, 2.0f);
	world.Add(b3);

	OurBody* b4 = b + 3;
	b4->Set(V2(0.5f, 0.5f), 25.0f);
	b4->position.Set(-5.5f, 2.0f);
	world.Add(b4);

	OurBody* b5 = b + 4;
	b5->Set(V2(1.0f, 1.0f), 100.0f);
	b5->position.Set(5.5f, 15.0f);
	world.Add(b5);

	numBodies += 5;

	j->Set(b1, b2, V2(0.0f, 1.0f));
	world.Add(j);

	numJoints += 1;
}

// A suspension bridge
static void Demo7(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	const int numPlanks = 15;
	float mass = 50.0f;

	for (int i = 0; i < numPlanks; ++i)
	{
		b->Set(V2(1.0f, 0.25f), mass);
		b->friction = 0.2f;
		b->position.Set(-8.5f + 1.25f * i, 5.0f);
		world.Add(b);
		++b; ++numBodies;
	}

	// Tuning
	float frequencyHz = 2.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stifness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	for (int i = 0; i < numPlanks; ++i)
	{
		j->Set(bodies+i, bodies+i+1, V2(-9.125f + 1.25f * i, 5.0f));
		j->softness = softness;
		j->biasFactor = biasFactor;

		world.Add(j);
		++j; ++numJoints;
	}

	j->Set(bodies + numPlanks, bodies, V2(-9.125f + 1.25f * numPlanks, 5.0f));
	j->softness = softness;
	j->biasFactor = biasFactor;
	world.Add(j);
	++j; ++numJoints;
}

// Dominos
static void Demo8(OurBody* b, OurJoint* j)
{
	OurBody* b1 = b;
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(V2(12.0f, 0.5f), FLT_MAX);
	b->position.Set(-1.5f, 10.0f);
	world.Add(b);
	++b; ++numBodies;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(V2(0.2f, 2.0f), 10.0f);
		b->position.Set(-6.0f + 1.0f * i, 11.125f);
		b->friction = 0.1f;
		world.Add(b);
		++b; ++numBodies;
	}

	b->Set(V2(14.0f, 0.5f), FLT_MAX);
	b->position.Set(1.0f, 6.0f);
	b->rotation = 0.3f;
	world.Add(b);
	++b; ++numBodies;

	OurBody* b2 = b;
	b->Set(V2(0.5f, 3.0f), FLT_MAX);
	b->position.Set(-7.0f, 4.0f);
	world.Add(b);
	++b; ++numBodies;

	OurBody* b3 = b;
	b->Set(V2(12.0f, 0.25f), 20.0f);
	b->position.Set(-0.9f, 1.0f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b1, b3, V2(-2.0f, 1.0f));
	world.Add(j);
	++j; ++numJoints;

	OurBody* b4 = b;
	b->Set(V2(0.5f, 0.5f), 10.0f);
	b->position.Set(-10.0f, 15.0f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b2, b4, V2(-7.0f, 15.0f));
	world.Add(j);
	++j; ++numJoints;

	OurBody* b5 = b;
	b->Set(V2(2.0f, 2.0f), 20.0f);
	b->position.Set(6.0f, 2.5f);
	b->friction = 0.1f;
	world.Add(b);
	++b; ++numBodies;

	j->Set(b1, b5, V2(6.0f, 2.6f));
	world.Add(j);
	++j; ++numJoints;

	OurBody* b6 = b;
	b->Set(V2(2.0f, 0.2f), 10.0f);
	b->position.Set(6.0f, 3.6f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b5, b6, V2(7.0f, 3.5f));
	world.Add(j);
	++j; ++numJoints;
}

// A multi-pendulum
static void Demo9(OurBody* b, OurJoint* j)
{
	b->Set(V2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);

	OurBody * b1 = b;
	++b;
	++numBodies;

	float mass = 10.0f;

	// Tuning
	float frequencyHz = 4.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stiffness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	const float y = 12.0f;

	for (int i = 0; i < 15; ++i)
	{
		V2 x(0.5f + i, y);
		b->Set(V2(0.75f, 0.25f), mass);
		b->friction = 0.2f;
		b->position = x;
		b->rotation = 0.0f;
		world.Add(b);

		j->Set(b1, b, V2(float(i), y));
		j->softness = softness;
		j->biasFactor = biasFactor;
		world.Add(j);

		b1 = b;
		++b;
		++numBodies;
		++j;
		++numJoints;
	}
}

void (*demos[])(OurBody* b, OurJoint* j) = {Demo1, Demo2, Demo3, Demo4, Demo5, Demo6, Demo7, Demo8, Demo9};
const char* demoStrings[] = {
	"Demo 1: A Single Box",
	"Demo 2: Simple Pendulum",
	"Demo 3: Varying Friction Coefficients",
	"Demo 4: Randomized Stacking",
	"Demo 5: Pyramid Stacking",
	"Demo 6: A Teeter",
	"Demo 7: A Suspension Bridge",
	"Demo 8: Dominos",
	"Demo 9: Multi-pendulum"};

static void InitDemo(int index)
{
	world.Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	demoIndex = index;
	demos[index](bodies, joints);
}

static void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS)
	{
		return;
	}

	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// Quit
		glfwSetWindowShouldClose(mainWindow, GL_TRUE);
		break;

	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		InitDemo(key - GLFW_KEY_1);
		break;

	case GLFW_KEY_A:
		OurWorld::accumulateImpulses = !OurWorld::accumulateImpulses;
		break;

	case GLFW_KEY_P:
		OurWorld::positionCorrection = !OurWorld::positionCorrection;
		break;

	case GLFW_KEY_W:
		OurWorld::warmStarting = !OurWorld::warmStarting;
		break;

	case GLFW_KEY_SPACE:
		LaunchBomb();
		break;
	
	// 일시정지 기능 추가 (s키를 누를 시)
	case GLFW_KEY_S:
		flag = !flag;
		break;

	// 빙판 기능 추가 (i키를 누를 시)
	case GLFW_KEY_I:
		Arbiter::flag2 = !Arbiter::flag2;
		break;
	}
}

static void Reshape(GLFWwindow*, int w, int h)
{
	width = w;
	height = h > 0 ? h : 1;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float aspect = float(width) / float(height);
	if (width >= height)
	{
		// aspect >= 1, set the height from -1 to 1, with larger width
		glOrtho(-zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);
	}
	else
	{
		// aspect < 1, set the width to -1 to 1, with larger height
		glOrtho(-zoom, zoom, -zoom / aspect + pan_y, zoom / aspect + pan_y, -1.0, 1.0);
	}
}

class MainClass {
public:
	static int mainFunc();
};


int MainClass::mainFunc()
{
	glfwSetErrorCallback(glfwErrorCallback);

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	mainWindow = glfwCreateWindow(width, height, "box2d-lite", NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);

	// Load OpenGL functions using glad
	int gladStatus = gladLoadGL();
	if (gladStatus == 0)
	{
		fprintf(stderr, "Failed to load OpenGL.\n");
		glfwTerminate();
		return -1;
	}

	glfwSwapInterval(1);
	glfwSetWindowSizeCallback(mainWindow, Reshape);
	glfwSetKeyCallback(mainWindow, Keyboard);

	float xscale, yscale;
	glfwGetWindowContentScale(mainWindow, &xscale, &yscale);
	float uiScale = xscale;

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsClassic();
	ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
	ImGui_ImplOpenGL2_Init();
	ImGuiIO& io = ImGui::GetIO();
	io.FontGlobalScale = uiScale;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	float aspect = float(width) / float(height);
	if (width >= height)
	{
		// aspect >= 1, set the height from -1 to 1, with larger width
		glOrtho(-zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);
	}
	else
	{
		// aspect < 1, set the width to -1 to 1, with larger height
		glOrtho(-zoom, zoom, -zoom / aspect + pan_y, zoom / aspect + pan_y, -1.0, 1.0);
	}

	InitDemo(0);

	while (!glfwWindowShouldClose(mainWindow))
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// Globally position text
		ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
		ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		ImGui::End();

		DrawText(5, 5, demoStrings[demoIndex]);
		DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

		char buffer[64];
		sprintf(buffer, "(A)ccumulation %s", OurWorld::accumulateImpulses ? "ON" : "OFF");
		DrawText(5, 65, buffer);

		sprintf(buffer, "(P)osition Correction %s", OurWorld::positionCorrection ? "ON" : "OFF");
		DrawText(5, 95, buffer);

		sprintf(buffer, "(W)arm Starting %s", OurWorld::warmStarting ? "ON" : "OFF");
		DrawText(5, 125, buffer);
		
		// 일시정지 문구 추가
		sprintf(buffer, "(S)top %s", flag ? "ON" : "OFF");
		DrawText(5, 155, buffer);

		// 빙판 문구 추가
		sprintf(buffer, "(I)ce plane %s", Arbiter::flag2 ? "ON" : "OFF");
		DrawText(5, 185, buffer);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		
		// flag에 따른 일시정지 여부
		if (flag == true)
		{
			world.Step(0);
		}
		else
		{
			world.Step(timeStep);
		}

		
		for (int i = 0; i < numBodies; ++i)
			DrawBody(bodies + i);

		for (int i = 0; i < numJoints; ++i)
			DrawJoint(joints + i);

		glPointSize(4.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_POINTS);
		std::map<ArbiterKey, Arbiter>::const_iterator iter;
		for (iter = world.arbiters.begin(); iter != world.arbiters.end(); ++iter)
		{
			const Arbiter& arbiter = iter->second;
			for (int i = 0; i < arbiter.numContacts; ++i)
			{
				V2 p = arbiter.contacts[i].position;
				glVertex2f(p.x, p.y);
			}
		}
		glEnd();
		glPointSize(1.0f);

		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwPollEvents();
		glfwSwapBuffers(mainWindow);
	}

	glfwTerminate();
	return 0;
}

int main() {
	MainClass::mainFunc();
}
