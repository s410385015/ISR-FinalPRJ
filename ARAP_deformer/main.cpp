#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <limits>
#include <Windows.h>
#include <atlstr.h>
#include <GL/glew.h>
#include <gl/GL.h>
#include <glut.h>
#include<tchar.h>
#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"
#include "Laplacian.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#if !defined(GLUT_WHEEL_UP)  
#  define GLUT_WHEEL_UP   3  
#  define GLUT_WHEEL_DOWN 4  
#endif  

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using namespace std;
using namespace Eigen;

static enum vexState{NON,ROI,HANDLES,OUTLIER};
// ----------------------------------------------------------------------------------------------------
// global variables

_GLMmodel *mesh;

int WindWidth, WindHeight;
int last_x , last_y;
int select_x, select_y;

typedef enum { SELECT_MODE, DEFORM_MODE ,DRAG_MODE,TEST_MODE} ControlMode;
ControlMode current_mode = SELECT_MODE;

vector<float*> colors;
vector<vector<int> > handles;
vector<vector<int>> adj;

bool deform_mesh_flag = false;
string displayLog = "SELECT_MODE";

vector<int> roi;
vector<vexState> visited;




vector<bool> outlier;
vector<int> handle;
vector<Vector3d> raw;


vector<Vector3d> current_mesh;

Laplacian *lp;

vector<int> _roi;
vector<int> indexInRoi;
vector<vector<int>> roi_adj;
bool ColorFlag = true;

double zFlag = -3.5;
double xFlag = 0;
double yFlag = 0;

struct Mouse
{
	int x;		/*	the x coordinate of the mouse cursor	*/
	int y;		/*	the y coordinate of the mouse cursor	*/
	int lmb;	/*	is the left button pressed?		*/
	int mmb;	/*	is the middle button pressed?	*/
	int rmb;	/*	is the right button pressed?	*/

	/*
	*	These two variables are a bit odd. Basically I have added these to help replicate
	*	the way that most user interface systems work. When a button press occurs, if no
	*	other button is held down then the co-ordinates of where that click occured are stored.
	*	If other buttons are pressed when another button is pressed it will not update these
	*	values.
	*
	*	This allows us to "Set the Focus" to a specific portion of the screen. For example,
	*	in maya, clicking the Alt+LMB in a view allows you to move the mouse about and alter
	*	just that view. Essentually that viewport takes control of the mouse, therefore it is
	*	useful to know where the first click occured....
	*/
	int xpress; /*	stores the x-coord of when the first button press occurred	*/
	int ypress; /*	stores the y-coord of when the first button press occurred	*/
};
typedef struct Mouse Mouse;
Mouse MousePos = { 0, 0, 0, 0, 0 };
typedef void(*ButtonCallback)();
struct Button
{
	int   x;							/* top left x coord of the button */
	int   y;							/* top left y coord of the button */
	int   w;							/* the width of the button */
	int   h;							/* the height of the button */
	int	  state;						/* the state, 1 if pressed, 0 otherwise */
	int	  highlighted;					/* is the mouse cursor over the control? */
	char* label;						/* the text label of the button */
	ButtonCallback callbackFunction;	/* A pointer to a function to call if the button is pressed */
};
typedef struct Button Button;

void Load(char*);
void Reset();
void Prepare();


void TheButtonCallback_Load()
{
	
	char filename[MAX_PATH];

	OPENFILENAME ofn;
	ZeroMemory(&filename, sizeof(filename));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;  // If you have a window to center over, put its HANDLE here
	ofn.lpstrFilter = "Obj Files\0*.obj\0Any File\0*.*\0";
	ofn.lpstrFile = filename;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = "Select a Mesh!";
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	
	if (GetOpenFileName(&ofn))
	{
		cout << filename << endl;
		Load(filename);
		Reset();
	}
}
Button MyButton = { 5, 5, 100, 50, 0, 0, "Load", TheButtonCallback_Load };


int mode = 0;
char* ModeName[3];
void TheButtonCallback_ChangeMode()
{
	mode = (mode + 1) % 3;
	switch (mode)
	{
		case 0:
		default:
			current_mode = SELECT_MODE;
			break;
		case 1:
			current_mode = DEFORM_MODE;
			break;
		case 2:
			current_mode = DRAG_MODE;
			Prepare();
			break;
	}
}
Button ModeButton = { 115, 5, 200, 50, 0, 0, "", TheButtonCallback_ChangeMode };


char* ColorName[2];
void TheButtonCallback_ShowColor()
{
	ColorFlag = !ColorFlag;
}
Button ColorButton = { 325, 5, 200, 50, 0, 0, "", TheButtonCallback_ShowColor };


void TheButtonCallback_Reset()
{
	Reset();
}
Button ResetButton = { 535, 5, 100, 50, 0, 0, "Reset", TheButtonCallback_Reset };

int ButtonClickTest(Button* b, int x, int y)
{
	
	if (b)
	{
		
		/*
		*	If clicked within button area, then return true
		*/
		if (x > b->x      &&
			x < b->x + b->w &&
			y > b->y      &&
			y < b->y + b->h) {
			return 1;
		}
	}

	/*
	*	otherwise false.
	*/
	return 0;
}
void ButtonRelease(Button *b, int x, int y)
{
	
	if (b)
	{
		/*
		*	If the mouse button was pressed within the button area
		*	as well as being released on the button.....
		*/
		if (ButtonClickTest(b, MousePos.xpress, MousePos.ypress) &&
			ButtonClickTest(b, x, y))
		{
			
			/*
			*	Then if a callback function has been set, call it.
			*/
			if (b->callbackFunction) {
			
				b->callbackFunction();
				
			}
		}

		/*
		*	Set state back to zero.
		*/
		b->state = 0;
	}
}
void ButtonPress(Button *b, int x, int y)
{
	if (b)
	{
		/*
		*	if the mouse click was within the buttons client area,
		*	set the state to true.
		*/
		if (ButtonClickTest(b, x, y))
		{
			b->state = 1;
		}
	}
}
void ButtonPassive(Button *b, int x, int y)
{
	if (b)
	{
		/*
		*	if the mouse moved over the control
		*/
		if (ButtonClickTest(b, x, y))
		{
			/*
			*	If the cursor has just arrived over the control, set the highlighted flag
			*	and force a redraw. The screen will not be redrawn again until the mouse
			*	is no longer over this control
			*/
			if (b->highlighted == 0) {
				b->highlighted = 1;
				glutPostRedisplay();
			}
		}
		else

			/*
			*	If the cursor is no longer over the control, then if the control
			*	is highlighted (ie, the mouse has JUST moved off the control) then
			*	we set the highlighting back to false, and force a redraw.
			*/
		if (b->highlighted == 1)
		{
			b->highlighted = 0;
			glutPostRedisplay();
		}
	}
}

void Font(void *font, char *text, int x, int y)
{
	glRasterPos2i(x, y);

	while (*text != '\0')
	{
		glutBitmapCharacter(font, *text);
		++text;
	}
}
void ButtonDraw(Button *b)
{
	int fontx;
	int fonty;

	if (b)
	{
		/*
		*	We will indicate that the mouse cursor is over the button by changing its
		*	colour.
		*/
		if (b->highlighted)
			glColor3f(0.7f, 0.7f, 0.8f);
		else
			glColor3f(0.6f, 0.6f, 0.6f);

		/*
		*	draw background for the button.
		*/
		glBegin(GL_QUADS);
		glVertex2i(b->x, b->y);
		glVertex2i(b->x, b->y + b->h);
		glVertex2i(b->x + b->w, b->y + b->h);
		glVertex2i(b->x + b->w, b->y);
		glEnd();

		/*
		*	Draw an outline around the button with width 3
		*/
		glLineWidth(3);

		/*
		*	The colours for the outline are reversed when the button.
		*/
		if (b->state)
			glColor3f(0.4f, 0.4f, 0.4f);
		else
			glColor3f(0.8f, 0.8f, 0.8f);

		glBegin(GL_LINE_STRIP);
		glVertex2i(b->x + b->w, b->y);
		glVertex2i(b->x, b->y);
		glVertex2i(b->x, b->y + b->h);
		glEnd();

		if (b->state)
			glColor3f(0.8f, 0.8f, 0.8f);
		else
			glColor3f(0.4f, 0.4f, 0.4f);

		glBegin(GL_LINE_STRIP);
		glVertex2i(b->x, b->y + b->h);
		glVertex2i(b->x + b->w, b->y + b->h);
		glVertex2i(b->x + b->w, b->y);
		glEnd();

		glLineWidth(1);


		/*
		*	Calculate the x and y coords for the text string in order to center it.
		*/
		fontx = b->x + (b->w - glutBitmapLength(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)b->label)) / 2;
		fonty = b->y + (b->h + 10) / 2;

		/*
		*	if the button is pressed, make it look as though the string has been pushed
		*	down. It's just a visual thing to help with the overall look....
		*/
		if (b->state) {
			fontx += 2;
			fonty += 2;
		}

		/*
		*	If the cursor is currently over the button we offset the text string and draw a shadow
		*/
		if (b->highlighted)
		{
			glColor3f(0, 0, 0);
			Font(GLUT_BITMAP_TIMES_ROMAN_24, b->label, fontx, fonty);
			fontx--;
			fonty--;
		}

		glColor3f(1, 1, 1);
		Font(GLUT_BITMAP_TIMES_ROMAN_24, b->label, fontx, fonty);
	}
}

void DrawUI()
{
	ButtonDraw(&MyButton);
	ModeButton.label = ModeName[mode];
	ButtonDraw(&ModeButton);

	int cindex = (ColorFlag) ? (0): (1);
	ColorButton.label = ColorName[cindex];
	ButtonDraw(&ColorButton);
	ButtonDraw(&ResetButton);
}

void Reset()
{
	roi.clear();
	handle.clear();
	roi_adj.clear();
	indexInRoi.clear();
	_roi.clear();
	for (int i = 0; i < mesh->numvertices; i++){
		visited[i] = NON;
		outlier[i] = false;
	}
	for (int vertIter = 0; vertIter < raw.size(); vertIter++)
	{
		mesh->vertices[3 * (vertIter+1)+0] = raw[vertIter][0];
		mesh->vertices[3 * (vertIter+1)+1] = raw[vertIter][1];
		mesh->vertices[3 * (vertIter+1)+2] = raw[vertIter][2];
		current_mesh[vertIter] = raw[vertIter];
	}
	ColorFlag = true;
	zFlag = -3.5;
	xFlag = 0;
	yFlag = 0;
	deform_mesh_flag = false;
	current_mode = SELECT_MODE;
	mode = 0;
}
void FindConnectivity(){
	adj.resize(mesh->numvertices);
	for (int i = 0; i < mesh->numtriangles; i++){
		for (int j = 0; j < 3; j++){
			for (int k = 0; k < 3; k++){

				if (j != k
					&&find(adj[mesh->triangles[i].vindices[j] - 1].begin(),
					adj[mesh->triangles[i].vindices[j] - 1].end(),
					mesh->triangles[i].vindices[k] - 1)
					== adj[mesh->triangles[i].vindices[j] - 1].end())
				{

					adj[(mesh->triangles[i].vindices[j]) - 1].push_back((mesh->triangles[i].vindices[k]) - 1);
				}

			}
		}
	}
}
void Prepare()
{
	if (handle.size() == 0)
		return;
	cout << "Prepare..." << endl;
	indexInRoi.resize(mesh->numvertices);
	int count = 0;

	roi_adj.resize(mesh->numvertices);
	for (int i = 0; i < mesh->numvertices; i++)
	{
		if (visited[i] != HANDLES && visited[i] != ROI)
			continue;
		for (int j = 0; j < adj[i].size(); j++)
		{

			if (visited[adj[i][j]] != NON){

				roi_adj[i].push_back(adj[i][j]);
			}

		}
	}

	for (int vertIter = 0; vertIter < mesh->numvertices; vertIter++)
	{
		if (visited[vertIter] == HANDLES){
			_roi.push_back(vertIter);
			indexInRoi[vertIter] = count++;
		}if (visited[vertIter] == ROI){
			if (roi_adj[vertIter].size() != adj[vertIter].size())
			{

				outlier[vertIter] = true;
				for (int i = 0; i < roi_adj[vertIter].size(); i++){
					if (visited[roi_adj[vertIter][i]] == ROI)
						outlier[roi_adj[vertIter][i]] = true;
				}

				handle.push_back(vertIter);
				visited[vertIter] = HANDLES;
				_roi.push_back(vertIter);
				indexInRoi[vertIter] = count++;
			}
		}
	}



	for (int vertIter = 0; vertIter < mesh->numvertices; vertIter++)
	{
		if (visited[vertIter] == ROI){
			_roi.push_back(vertIter);
			indexInRoi[vertIter] = count++;
		}
	}




	lp = new Laplacian();

	lp->PrepareDeform(handle.size(), _roi, roi_adj, current_mesh, indexInRoi);
	cout << "PrepareDone!" << endl;
}

void Reshape(int width, int height)
{

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	
	int base = min(width , height);

	tbReshape(width, height);
	glViewport(0 , 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0,(GLdouble)width / (GLdouble)height , 1.0, 128.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(xFlag, yFlag, zFlag);

	WindWidth = width;
	WindHeight = height;
	//glRotatef(180, 0, 1, 0);
}
void Display(void)
{
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glPushMatrix();
	tbMatrix();

	

	// render solid model
	glEnable(GL_LIGHTING);
	glColor3f(1.0 , 1.0 , 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
	glmDraw(mesh , GLM_SMOOTH);

	// render wire model
	glPolygonOffset(1.0 , 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glLineWidth(1.0f);
	
	/*
	glColor3f(0.6 , 0.0 , 0.8);
	glColor3f(0.0, 0.0, 0.0);
	glPolygonMode(GL_FRONT_AND_BACK , GL_LINE);
	glmDraw(mesh , GLM_SMOOTH);
	*/


	// render handle points

	if (ColorFlag){
		glPointSize(5.0);
		glEnable(GL_POINT_SMOOTH);
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);



		glColor3f(0.0, 1.0, 0.0);
		for (int handleIter = 0; handleIter < handle.size(); handleIter++)
		{
			int idx = handle[handleIter] + 1;
			glVertex3fv((float *)&mesh->vertices[3 * idx]);
		}

		glColor3f(1.0, 0.0, 0.0);

		for (int handleIter = 0; handleIter < roi.size(); handleIter++)
		{
			int idx = roi[handleIter] + 1;
			glVertex3fv((float *)&mesh->vertices[3 * idx]);
		}

		glEnd();
	}
	
	
	glPopMatrix();


	
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	/*
	*	Set the orthographic viewing transformation
	*/
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 800, 800, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//output();
	DrawUI();
	
	Reshape(800, 800);
	

	glFlush();  
	
	glutSwapBuffers();
}
vector3 Unprojection(vector2 _2Dpos)
{
	
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x , viewport[3] - (int)_2Dpos.y , 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = {0.0 , 0.0 , 0.0};

	gluUnProject(X , ((double)viewport[3] - Y) , (double)Depth , ModelViewMatrix , ProjectionMatrix , viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector3(wpos[0] , wpos[1] , wpos[2]);
}
vector2 projection_helper(vector3 _3Dpos)
{
	
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	double wpos[3] = {0.0 , 0.0 , 0.0};
	gluProject(_3Dpos.x, _3Dpos.y, _3Dpos.z, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector2(wpos[0], (double)viewport[3]-wpos[1]);
}


void mouse(int button, int state, int x, int y)
{
	tbMouse(button, state, x, y);
	MousePos.x = x;
	MousePos.y = y;

	if (state == GLUT_DOWN)
	{
		/*
		*	This holds the location of the first mouse click
		*/
		if (!(MousePos.lmb || MousePos.mmb || MousePos.rmb)) {
			MousePos.xpress = x;
			MousePos.ypress = y;

		}

		/*
		*	Which button was pressed?
		*/
		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			MousePos.lmb = 1;
			ButtonPress(&MyButton, x, y);
			ButtonPress(&ModeButton, x, y);
			ButtonPress(&ColorButton, x, y);
			ButtonPress(&ResetButton, x, y);
			break;
		case GLUT_MIDDLE_BUTTON:
			MousePos.mmb = 1;
			break;
		case GLUT_RIGHT_BUTTON:
			MousePos.rmb = 1;
			break;
		}
	}
	else
	{
		/*
		*	Which button was released?
		*/
		switch (button)
		{
		case GLUT_LEFT_BUTTON:
			MousePos.lmb = 0;
			ButtonRelease(&MyButton, x, y);
			ButtonRelease(&ModeButton, x, y);
			ButtonRelease(&ColorButton, x, y);
			ButtonRelease(&ResetButton, x, y);
			break;
		case GLUT_MIDDLE_BUTTON:
			MousePos.mmb = 0;
			break;
		case GLUT_RIGHT_BUTTON:
			MousePos.rmb = 0;
			break;
		}
	}
	glutPostRedisplay();


	if( current_mode==SELECT_MODE && button==GLUT_RIGHT_BUTTON )
	{
		if(state==GLUT_DOWN)
		{
			select_x = x;
			select_y = y;
		}
		else
		{
		
			for (int vertIter = 1; vertIter<=mesh->numvertices; vertIter++)
			{
				vector3 pt(mesh->vertices[3 * vertIter + 0], mesh->vertices[3 * vertIter + 1], mesh->vertices[3 * vertIter + 2]);
				vector2 pos = projection_helper(pt);

				// if the projection is inside the box specified by mouse click&drag, add it to current handle
				if (pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y&&visited[vertIter-1]==NON)
				{
					roi.push_back(vertIter-1);
					visited[vertIter-1] = ROI;
					
				}
				
			}
			
		
			
		}
	}
	// select handle
	else if (current_mode == DEFORM_MODE && button == GLUT_RIGHT_BUTTON && roi.empty() == false)
	{
		if (state == GLUT_DOWN)
		{
			select_x = x;
			select_y = y;
		}
		else{
			for (int vertIter = 1; vertIter <= mesh->numvertices; vertIter++)
			{
				vector3 pt(mesh->vertices[3 * vertIter + 0], mesh->vertices[3 * vertIter + 1], mesh->vertices[3 * vertIter + 2]);
				vector2 pos = projection_helper(pt);

				// if the projection is inside the box specified by mouse click&drag, add it to current handle
				if (pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y && visited[vertIter-1] == ROI)
				{
					handle.push_back(vertIter - 1);
					visited[vertIter - 1] = HANDLES;

				}

			}
		}

		
	}
	
	else if (current_mode == DRAG_MODE && button == GLUT_RIGHT_BUTTON)
	{
		deform_mesh_flag = true;
	}
	if(button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
		deform_mesh_flag = false;

	last_x = x;
	last_y = y;
}
void motion(int x, int y)
{
	tbMotion(x, y);

	// if in deform mode and a handle is selected, deform the mesh
	if (current_mode == DRAG_MODE &&deform_mesh_flag == true)
	{
		
		matrix44 m;
		vector4 vec = vector4((float)(x - last_x) / 1000.0f , (float)(y - last_y) / 1000.0f , 0.0 , 1.0);

		gettbMatrix((float *)&m);
		vec = m * vec;
		
		// deform handle points
		for(int vertIter=0; vertIter<handle.size(); vertIter++)
		{
			int idx = handle[vertIter]+1;
			Vector3d pt(mesh->vertices[3*idx+0]+vec.x, mesh->vertices[3*idx+1]-vec.y, mesh->vertices[3*idx+2]-vec.z);
			
			//Vector3d pt(raw[(idx - 1)][0] + vec.x, raw[(idx - 1)][1] - vec.y, raw[ (idx - 1)][2]-vec.z);
			//mesh->vertices[3 * idx + 0] = pt[0];
			//mesh->vertices[3 * idx + 1] = pt[1];
			//mesh->vertices[3 * idx + 2] = pt[2];
			if (outlier[idx-1] == true)
				continue;
			current_mesh[idx-1] = pt;
			
		}
		
		
		

		
		
		
		MatrixXd result=lp->DoDeformation(handle.size(),_roi,roi_adj,current_mesh,indexInRoi);
		

		
		for (int vertIter = 0; vertIter < _roi.size(); vertIter++)
		{
			int idx = _roi[vertIter]+1;

			if (outlier[idx-1] == true)
				continue;
		
			mesh->vertices[3 * idx + 0] = result(vertIter * 3, 0);
			mesh->vertices[3 * idx + 1] = result(vertIter * 3 + 1, 0);
			mesh->vertices[3 * idx + 2] = result(vertIter * 3 + 2, 0);
			//current_mesh[idx - 1] = Vector3d(result(vertIter * 3, 0), result(vertIter * 3 + 1, 0), result(vertIter * 3 + 2, 0));
			
		}
		
	}

	last_x = x;
	last_y = y;
}
void keyboard(unsigned char key, int x, int y )
{

	switch(key)
	{
		case 't':
			current_mode = TEST_MODE;
			displayLog = "TEST_MODE";
			break;
		case 'd':
			cout <<"?" <<endl;
			current_mode = DEFORM_MODE;
			displayLog = "DEFORM_MODE";
			break;
		default:
		case 's':
			current_mode = SELECT_MODE;
			displayLog = "SELECT_MODE";
			break;
		case '+':
			zFlag -= 0.1;		
			break;
		case '-':
			zFlag += 0.1;
			break;
		case '8':
			yFlag += 0.05;
			break;
		case '2':
			//glTranslatef(0.0, -0.05, 0.0);
			yFlag -= 0.05;
			break;
		case '4':
			xFlag-=0.05;
			break;
		case '6':
			xFlag += 0.05;
			break;
		case 'r':
			Reset();
			break;
		case 'p':
			current_mode = DRAG_MODE;
			Prepare();

			break;
		case 'o':
			ColorFlag = !ColorFlag;
			break;
	}
}
void MousePassiveMotion(int x, int y)
{
	/*
	*	Calculate how much the mouse actually moved
	*/
	int dx = x - MousePos.x;
	int dy = y - MousePos.y;

	/*
	*	update the mouse position
	*/
	MousePos.x = x;
	MousePos.y = y;

	/*
	*	Check MyButton to see if we should highlight it cos the mouse is over it
	*/
	ButtonPassive(&MyButton, x, y);
	ButtonPassive(&ModeButton, x, y);
	ButtonPassive(&ColorButton, x, y);
	ButtonPassive(&ResetButton, x, y);
	/*
	*	Note that I'm not using a glutPostRedisplay() call here. The passive motion function
	*	is called at a very high frequency. We really don't want much processing to occur here.
	*	Redrawing the screen every time the mouse moves is a bit excessive. Later on we
	*	will look at a way to solve this problem and force a redraw only when needed.
	*/
}

void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
}

void Load(char* path){

	raw.clear();
	current_mesh.clear();
	visited.clear();
	outlier.clear();
	adj.clear();

	mesh = glmReadOBJ(path);
	for (int i = 1; i <= mesh->numvertices; i++){
		raw.push_back(Vector3d(mesh->vertices[i * 3], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]));
		current_mesh.push_back(Vector3d(mesh->vertices[i * 3], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]));
	}
	glmUnitize(mesh);
	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);
	FindConnectivity();
	for (int i = 0; i < mesh->numvertices; i++){
		visited.push_back(NON);
		outlier.push_back(false);
		
	}
	Reshape(800, 800);
	current_mode = SELECT_MODE;
}


int main(int argc, char *argv[])
{
	
	WindWidth = 800;
	WindHeight = 800;

	GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
	GLfloat light_diffuse[] = {0.8, 0.8, 0.8, 1.0};
	GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat light_position[] = {0.0, 0.0, 1.0, 0.0};

	// color list for rendering handles
	float red[] = {1.0, 0.0, 0.0};
	colors.push_back(red);
	float yellow[] = {1.0, 1.0, 0.0};
	colors.push_back(yellow);
	float blue[] = {0.0, 1.0, 1.0};
	colors.push_back(blue);
	float green[] = {0.0, 1.0, 0.0};
	colors.push_back(green);

	glutInit(&argc, argv);
	glutInitWindowSize(WindWidth, WindHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("Laplacian");

	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutPassiveMotionFunc(MousePassiveMotion);
	glClearColor(0, 0, 0, 0);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	tbInit(GLUT_LEFT_BUTTON);
	tbAnimate(GL_TRUE);

	glutTimerFunc(40, timf, 0); // Set up timer for 40ms, about 25 fps
	ModeName[0] = "Select Roi";
	ModeName[1] = "Seletct Handle";
	ModeName[2] = "Deform";
	ColorName[0] = "Show";
	ColorName[1] = "Hide";
	// load 3D model
	Load("../data/torus_cloud.obj");
	
	glutMainLoop();

	return 0;

}