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

// Texture visualization
#include "texture.h"
static Renderer* renderer; //this is a static pointer to a Renderer used in the glut callback functions
static GLUquadricObj* q; // water object

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

#define R 0.4 // bowl radius (for visualization ONLY) was 0.4
#define radius 0.05 // ball radius (for visualization ONLY) was 0.05
#define RR (R - radius) // radius to center of ball (for visualization ONLY)

// Initialize Ball-in-Bowl Class
BallinBowl sys(1.0, 0.0, -9.81, len, deltaT); // m, B, g, L, dt in seconds

// Initialize iSAC Class
double percent_height = 0.3; // Allowed ball height as a percentage of bowl radius
iSAC controller(sys, -UMAX, UMAX, percent_height * len); // Ball-in-bowl class object, min, max

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
//                     D R A W   W A T E R
//
// This Function Is Called To Draw The Water In The Bowl
//---------------------------------------------------------------------
void DrawWater(void)
{
	const GLint numberOfSides = 360;
	const GLint numberOfVertices = numberOfSides + 2;

	GLfloat circleVerticesX[numberOfVertices];
	GLfloat circleVerticesY[numberOfVertices];
	GLfloat circleVerticesZ[numberOfVertices];

	// DON'T CHANGE must all be 0
	circleVerticesX[0] = 0;
	circleVerticesY[0] = 0;
	circleVerticesZ[0] = 0;

	//double r = sqrt(pow(R_vis,2) + pow((1.0-percent_height)*R_vis,2));
	double r = 0.05; 
	for (int i = 1; i < numberOfVertices; i++)
	{
		circleVerticesX[i] = r * cos(i * 2 * M_PI / numberOfSides);
		circleVerticesY[i] = r * sin(i * 2 * M_PI / numberOfSides);
		circleVerticesZ[i] = circleVerticesZ[0];
	}

	// combine X, Y, Z coordinates to single array
	GLfloat allCircleVertices[numberOfVertices * 3];

	for (int i = 0; i < numberOfVertices; i++)
	{
		allCircleVertices[i * 3] = circleVerticesX[i];
		allCircleVertices[(i * 3) + 1] = circleVerticesY[i];
		allCircleVertices[(i * 3) + 2] = circleVerticesZ[i];
	}

	glPushMatrix();
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnable(GL_BLEND);
	glColor4f(0.31f, 0.54f, 0.95f, 0.35f);

	if (!q)
	{
		q = gluNewQuadric();
		gluQuadricDrawStyle(q, GLU_FILL);
	}

	double ball_angle_x = fmod(sys.Xcurr[2] * 180 / M_PI, 360);
	double ball_angle_y = -1 * fmod(sys.Xcurr[0] * 180 / M_PI, 360);
	double water_angle_limit = 60.0; // ADJUSTABLE max angle that the water can reach w.r.t. the x-y plane

	// set limits on water angle about the x-axis
	if ((ball_angle_x > water_angle_limit && ball_angle_x < 180.0) || (ball_angle_x <= -180.0 && ball_angle_x > -360 + water_angle_limit))
	{
		ball_angle_x = water_angle_limit;
	}
	else if ((ball_angle_x >= 180.0 && ball_angle_x < 360.0 - water_angle_limit) || (ball_angle_x < -water_angle_limit && ball_angle_x > -180.0))
	{
		ball_angle_x = -water_angle_limit;
	}
	else
	{
		ball_angle_x = fmod(sys.Xcurr[2] * 180 / M_PI, 360);
	}

	// set limits on water angle about the y-axis
	if ((ball_angle_y > water_angle_limit && ball_angle_y < 180.0) || (ball_angle_y <= -180.0 && ball_angle_y > -360 + water_angle_limit))
	{
		ball_angle_y = water_angle_limit;
	}
	else if ((ball_angle_y >= 180.0 && ball_angle_y < 360.0 - water_angle_limit) || (ball_angle_y < -water_angle_limit && ball_angle_y > -180.0))
	{
		ball_angle_y = -water_angle_limit;
	}
	else
	{
		ball_angle_y = -1 * fmod(sys.Xcurr[0] * 180 / M_PI, 360);
	}

	// draw water hemisphere   
	glTranslatef(CurrentPosition[PosX], CurrentPosition[PosY], R);
	glRotatef(ball_angle_y, 0.0, 1.0, 0.0);
	glRotatef(ball_angle_x, 1.0, 0.0, 0.0);
	glEnable(GL_CLIP_PLANE0);
	double clipEq[4] = { 0.0, 0.0, -1.0, -(1 - percent_height) * R }; // ADJUSTABLE plane that removes the top half of the sphere of water
	glClipPlane(GL_CLIP_PLANE0, clipEq);
	gluSphere(q, R, 16, 16);
	glDisable(GL_CLIP_PLANE0);

	// draw water surface
	glTranslatef(0.0, 0.0, -(1 - percent_height) * R); // z offset from bowl sphere center
	glVertexPointer(3, GL_FLOAT, 0, allCircleVertices);
	glDrawArrays(GL_TRIANGLE_FAN, 0, numberOfVertices);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisable(GL_BLEND);
	glPopMatrix();
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

	renderer->DrawTable();
	//DrawTable();
	DrawBall();
	DrawBowl();
	DrawWater();

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

	float offset = 1.0f;
	renderer->t += offset;

	// Check Joystick Connection
	if (!glfwJoystickPresent(GLFW_JOYSTICK_1)) {

		std::cout << "Joystick Disconnected " << std::endl;
	}

	// Measure Desired Action
	int axesCount;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axesCount);
	CurrentJoystick[PosX] = axes[1] * UMAX;
	CurrentJoystick[PosY] = axes[0] * UMAX;

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

		sys.step();

		CurrentPosition[PosX] = sys.Xcurr[4];
		CurrentPosition[PosY] = sys.Xcurr[6];
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

	renderer = new Renderer;
	renderer->init();

	glutTimerFunc(50, TimerCB, 1); // 50 msec time step = 20 Hz

	glutMainLoop();

	glfwTerminate();

	return 0;
}