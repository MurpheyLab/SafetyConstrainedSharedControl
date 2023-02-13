//---------------------------------------------------------------------
// ballinbowl_joystick.cpp
// Interactive bowl visualization moved using a commercial joystick 
// controller.
//---------------------------------------------------------------------

// Custom Headers
#include "SCSC.hpp"

// Standard Headers & Libraries
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES // this has to come before including the math header file
#include <math.h> // sine, cosine
#include "cmath"
#include <cstdlib>
#include <iostream> // cin
#include <fstream>
#include <sstream>
#include <ctime>
#include <glut.h>
#include <glfw3.h>
using namespace std;
//const double PI = 3.1415926535987;

// Matrix math
#include<Eigen>
using Eigen::MatrixXd;

// Data Logging
string path = "Trial.csv";
ofstream logfile;

// Timing
#include <chrono>
using namespace std::chrono;

// Screen Dimension Constants (OpenGL Visualization)
const int SCREEN_WIDTH = 1500; //4000
const int SCREEN_HEIGHT = 750; //2000

// Global Parameters & Variables
#define PosX 0
#define PosY 1
#define PosZ 2

#define TF 30.0 // trial length (seconds)
#define deltaT 0.05 // (20 Hz)
#define UMAX 20 // max control effort
#define len 0.249 // pendulum length for simulation

#define R 0.4 // bowl radius (for visualization ONLY)
#define radius 0.05 // ball radius (for visualization ONLY)
#define RR (R - radius) // radius to center of ball (for visualization ONLY)

// Initialize Ball-in-Bowl Class
BallinBowl sys(1.0, 0.0, -9.81, len, deltaT); // m, B, g, L, dt in seconds

// Initialize iSAC Class
iSAC controller(sys, -UMAX, UMAX, 0.3 * len); // Ball-in-bowl class object, min, max

float time_current = 0.0;
float trial_flag = 0.0; 

double CurrentPosition[3];
GLfloat BallPosition[3];
double CurrentJoystick[2];
Vector2d u_desired;
Vector8d x_current;
float ball_height;

//---------------------------------------------------------------------
//                  O P E N G L   M A T E R I A L S
//---------------------------------------------------------------------
// General OpenGL Material Parameters
GLfloat Specular[] = { 1.00, 1.00, 1.00, 1.00 };
GLfloat Emissive[] = { 0.00, 0.00, 0.00, 1.00 };
GLfloat Shininess = { 128.00 };
GLfloat SpecularOff[] = { 0.00, 0.00, 0.00, 0.00 };
GLfloat EmissiveOff[] = { 0.50, 0.50, 0.50, 0.00 };
GLfloat ShininessOff = { 0.00 };

//---------------------------------------------------------------------
//                     D R A W   T A B L E
//
// This Function Is Called To Draw A Plane
//---------------------------------------------------------------------
void DrawTable(void)
{
	int i, j;
	GLfloat v[4][3];
	GLfloat width = 4.0, length = 2.5; 

	for (i = 0; i < 4; ++i) { // level with x-y plane
		v[i][2] = 0;
	}

	v[0][0] = -width;
	v[0][1] = length;
	v[1][0] = width;
	v[1][1] = length;
	v[2][0] = width;
	v[2][1] = -length;
	v[3][0] = -width;
	v[3][1] = -length;

	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_QUADS);
	glVertex3fv(v[0]);
	glVertex3fv(v[1]);
	glVertex3fv(v[2]);
	glVertex3fv(v[3]);
	glEnd();
}

//---------------------------------------------------------------------
//                     D R A W   B A L L
//
// This Function Is Called To Draw The Ball
//---------------------------------------------------------------------
void DrawBall(void)
{
	BallPosition[PosX] = CurrentPosition[PosX] + RR * sin(sys.Xcurr[0]) * cos(sys.Xcurr[2]);
	BallPosition[PosY] = CurrentPosition[PosY] + RR * sin(sys.Xcurr[2]) * cos(sys.Xcurr[0]);
	BallPosition[PosZ] = RR - RR * cos(sys.Xcurr[0]) * cos(sys.Xcurr[2]);

	// Set ball color based on safety criterion
	if (len * (1 - (cos(sys.Xcurr[0]) * cos(sys.Xcurr[2]))) > (0.3 * len)) // check if ball is above max height
	{
		glColor3f(1.0f, 0.0f, 0.0f); // red
		std::cout << "Failure" << std::endl;
	}
	else
	{
		glColor3f(1.0f, 1.0f, 0.0f); // yellow
	}

	glPushMatrix();
	glTranslatef(BallPosition[PosX], BallPosition[PosY], BallPosition[PosZ] + radius);
	glutSolidSphere(radius, 20, 20);
	glPopMatrix();
}

//---------------------------------------------------------------------
//                     D R A W   B O W L
//
// This Function Is Called To Draw The Bowl
//---------------------------------------------------------------------
void DrawBowl(void)
{
	int i, j;
	GLfloat r = R;
	// change these to render more or less circles
	const int scalex = 30;
	const int scaley = 30;
	GLfloat v[scalex * scaley][3];

	for (i = 0; i < scalex; ++i)
	{
		for (j = 0; j < scaley; ++j)
		{
			v[i * scaley + j][0] = CurrentPosition[PosX] + r * cos(j * 2 * M_PI / scaley) * cos(i * M_PI / (2 * scalex));
			v[i * scaley + j][1] = CurrentPosition[PosY] + r * sin(j * 2 * M_PI / scaley) * cos(i * M_PI / (2 * scalex));
			v[i * scaley + j][2] = r - r * sin(i * M_PI / (2 * scalex));
		}
	}

	glColor3f(0.3f, 0.0f, 0.51f); // purple

	glBegin(GL_LINES);
	for (i = 10; i < scalex - 1; ++i) // i = 1 for full bowl (half sphere)
	{
		for (j = 0; j < scaley; ++j)
		{
			glVertex3fv(v[i * scaley + j]);
			glVertex3fv(v[i * scaley + (j + 1) % scaley]);
			glVertex3fv(v[(i + 1) * scaley + (j + 1) % scaley]);
			glVertex3fv(v[(i + 1) * scaley + j]);
		}
	}
	glEnd();

}

//---------------------------------------------------------------------
//                    I N I T   O P E N G L
//
// This Function Initializes the OpenGl Graphics Engine
//---------------------------------------------------------------------
void InitOpenGl(void)
{
	glShadeModel(GL_SMOOTH);
	glLoadIdentity();
	GLfloat GrayLight[] = { 0.75, 0.75, 0.75, 1.0 };
	GLfloat LightPosition[] = { 1.0, 2.0, 1.0, 0.0 };
	GLfloat LightDirection[] = { 0.0, 0.0, -1.0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, GrayLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, GrayLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, GrayLight);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glClearColor(1.0, 1.0, 1.0, 1.0); // white background
}

//---------------------------------------------------------------------
//                         D I S P L A Y
//
// This Function Is Called By OpenGL To Redraw The Scene
// Here's Where You Put The End Effector And Block Drawing FuntionCalls
//---------------------------------------------------------------------
void Display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	//gluLookAt(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // for top down view 
	//gluLookAt(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // for participant view point
	//gluLookAt(0.5, 0.0, 0.5, -0.2, 0.0, 0.0, 0.0, 0.0, 1.0); // for participant view point
	//gluLookAt(0.645, 0.0, 0.5, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0); // for participant view point

	glutPostRedisplay();
	gluLookAt(10.2, 0.0, 5.1, -0.015, 0.0, 0.0, 0.0, 0.0, 1.0);

	DrawTable();
	DrawBall();
	DrawBowl();

	glPopMatrix();
	glutSwapBuffers();
}

//---------------------------------------------------------------------
//                          R E S H A P E
//
// The Function Is Called By OpenGL Whenever The Window Is Resized
//---------------------------------------------------------------------
void Reshape(int iWidth, int iHeight)
{
	glViewport(0, 0, (GLsizei)iWidth, (GLsizei)iHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float fAspect = (float)iWidth / iHeight;
	gluPerspective(30.0, fAspect, 0.05, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

//---------------------------------------------------------------------
//                         K E Y B O A R D
//
// This Function Is Called By OpenGL Whenever A Key Was Hit
//---------------------------------------------------------------------
void Keyboard(unsigned char ucKey, int iX, int iY)
{
	switch (ucKey)
	{
	case 's': // Start
		// Start trial and allow movement 
		printf("S pressed: Trial and timer starting...\n");
		time_current = 0.0;
		trial_flag = 1.0;
		break;
	case 27: // ESC key pressed - stop motion and simulation
		printf("ESC pressed: exit routine starting \n");
		trial_flag = 0.0;
		exit(0);
		break;
	}
}

//---------------------------------------------------------------------
//                        T I M E R   C B
//
// This Is A Timer Function Which Calls The HapticMASTER To Get The
// New EndEffector Position and Updates The Bowl and Ball Position
//---------------------------------------------------------------------
void TimerCB(int iTimer)
{

	// Check Joystick Connection
	if (!glfwJoystickPresent(GLFW_JOYSTICK_1)) {

		std::cout << "Joystick Disconnected " << std::endl;
	}

	// Measure Desired Action
	int axesCount;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axesCount);
	CurrentJoystick[PosX] = axes[1] * UMAX;
	CurrentJoystick[PosY] = axes[0] * UMAX;
	//printf("Joystick: %f, %f \n", CurrentJoystick[PosX], CurrentJoystick[PosY]);

	// Wait for Start
	if (trial_flag == 0.0) {
		CurrentJoystick[PosX] = 0.0;
		CurrentJoystick[PosY] = 0.0;
	}

	// Send Control to System
	if (trial_flag == 1.0) {

		u_desired(0) = CurrentJoystick[PosX];
		u_desired(1) = CurrentJoystick[PosY];

		x_current << sys.Xcurr[0], sys.Xcurr[1], sys.Xcurr[2], sys.Xcurr[3], sys.Xcurr[4], sys.Xcurr[5], sys.Xcurr[6], sys.Xcurr[7];

		Vector2d temp = controller.predict(x_current, u_desired); // SCSC algorithm call
		sys.Ucurr = { temp(0), temp(1) };

		logfile << time_current << "," << sys.Xcurr[0] << "," << sys.Xcurr[1] << "," << sys.Xcurr[2] << "," << sys.Xcurr[3] << "," << sys.Xcurr[4] << "," << sys.Xcurr[5] << "," << sys.Xcurr[6] << "," << sys.Xcurr[7] << "," << sys.Ucurr[0] << "," << sys.Ucurr[1] << "," << u_desired[0] << "," << u_desired[1] << "\n";

		time_current = time_current + deltaT;
		//printf("Time: %f \n", time_current);

		sys.step();

		CurrentPosition[PosX] = sys.Xcurr[4];
		CurrentPosition[PosY] = sys.Xcurr[6];
		//printf("X,Y: %f, %f \n", sys.Xcurr[4], sys.Xcurr[6]);
	}

	// Set end time of trial 
	if (time_current > TF + deltaT) {
		logfile.close();
		exit(0);
	}

	glfwPollEvents();
	glutTimerFunc(50, TimerCB, 1); // 50 msec time step = 20 Hz

}

//---------------------------------------------------------------------
//                            M A I N
//---------------------------------------------------------------------
int main(int argc, char** argv)
{
	// Open Log File
	logfile.open(path);

	// Initial Conditions
	CurrentPosition[PosX] = 0.0; 
	CurrentPosition[PosY] = 0.0; 
	CurrentPosition[PosZ] = 0.0;
	sys.Xcurr = { 0.0, 0.0, 0.0, 0.0, CurrentPosition[PosX], 0.0, CurrentPosition[PosY], 0.0 }; 
	x_current << sys.Xcurr[0], sys.Xcurr[1], sys.Xcurr[2], sys.Xcurr[3], sys.Xcurr[4], sys.Xcurr[5], sys.Xcurr[6], sys.Xcurr[7];

	printf("Visualization Starting... \n");

	// OpenGL Initialization Calls
	glutInit(&argc, argv);
	glfwInit();
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);

	// Create The OpenGL Window and Initialize Visualization Functions
	glutCreateWindow("Bowl-in-Ball");
	InitOpenGl();
	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutKeyboardFunc(Keyboard);

	glutTimerFunc(50, TimerCB, 1); // 50 msec time step = 20 Hz

	glutMainLoop();

	glfwTerminate();

	return 0;
}