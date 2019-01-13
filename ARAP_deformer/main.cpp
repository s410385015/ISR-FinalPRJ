#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <limits>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>

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
int selected_handle_id = -1;
bool deform_mesh_flag = false;
string displayLog = "SELECT_MODE";

vector<int> roi;
vector<vexState> visited;
vector<int> test;


vector<bool> outlier;
vector<int> handle;
vector<Vector3d> raw;


vector<Vector3d> current_mesh;

Laplacian *lp;

vector<int> _roi;
vector<int> indexInRoi;
vector<vector<int>> roi_adj;
bool ColorFlag = true;



void init()
{
	for (int i = 0; i < mesh->numvertices; i++){
		visited[i] = NON;
		outlier[i] = false;
	}
	for (int vertIter = 0; vertIter < raw.size(); vertIter++)
	{
		mesh->vertices[3 * (vertIter+1)+0] = raw[vertIter][0];
		mesh->vertices[3 * (vertIter+1)+1] = raw[vertIter][1];
		mesh->vertices[3 * (vertIter+1)+2] = raw[vertIter][2];

	}
	roi.clear();
	handle.clear();
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

void output()
{
	const char* str = displayLog.c_str();

	glColor3f(0.0f, 1.0f, 0.0f);
	//float w=(float)glutBitmapLength(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned char*)str)/ (float)glutGet(GLUT_WINDOW_WIDTH);
	glRasterPos2f(0.5, 0.5);
	int len, i;
	len = (int)strlen(str);
	for (i = 0; i < len; i++) {
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
	}
}

void Reshape(int width, int height)
{
	
	int base = min(width , height);

	tbReshape(width, height);
	glViewport(0 , 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0,(GLdouble)width / (GLdouble)height , 1.0, 128.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -3.5);

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


		glColor3f(0.0,0.0, 1.0);
		for (int handleIter = 0; handleIter < test.size(); handleIter++)
		{
			if (test[handleIter] == 1){
			
				int idx = handleIter + 1;
				glVertex3fv((float *)&mesh->vertices[3 * idx]);
			}
		}


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
	output();
	glFlush();  
	
	glutSwapBuffers();
}

// ----------------------------------------------------------------------------------------------------
// mouse related functions

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
	else if (current_mode == TEST_MODE && button == GLUT_RIGHT_BUTTON && roi.empty() == false)
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
				if (pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y && visited[vertIter - 1] == ROI)
				{
					test[vertIter - 1] = 1;
				
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

void Prepare()
{
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
					if (visited[roi_adj[vertIter][i]]==ROI)
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
	cout << handle.size() << endl;
	lp->PrepareDeform(handle.size(), _roi, roi_adj, current_mesh, indexInRoi);
	cout << "PrepareDone!" << endl;
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
		cout << endl;
	}

	last_x = x;
	last_y = y;
}

// ----------------------------------------------------------------------------------------------------
// keyboard related functions

void keyboard(unsigned char key, int x, int y )
{

	switch(key)
	{
		case 't':
			current_mode = TEST_MODE;
			displayLog = "TEST_MODE";
			break;
		case 'd':
			current_mode = DEFORM_MODE;

			displayLog = "DEFORM_MODE";
			break;
		default:
		case 's':
			current_mode = SELECT_MODE;
			displayLog = "SELECT_MODE";
			break;
		case '+':
			
			glTranslatef(0.0, 0.0, 0.1f);
			break;
		case '-':
			glTranslatef(0.0, 0.0, -0.1f);
			break;
		case '8':
			glTranslatef(0.0, 0.05, 0.0);
			break;
		case '2':
			glTranslatef(0.0, -0.05, 0.0);
		
			break;
		case '4':
			glTranslatef(0.05,0.0 , 0.0);

			break;
		case '6':
			glTranslatef(-0.05,0.0, 0.0);

			break;
		case 'r':
			init();
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

// ----------------------------------------------------------------------------------------------------
// main function

void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
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
	glutCreateWindow("ARAP");

	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
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

	// load 3D model
	mesh = glmReadOBJ("../data/man.obj");
	for (int i = 1; i <= mesh->numvertices; i++){
		raw.push_back(Vector3d(mesh->vertices[i * 3], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]));
		current_mesh.push_back(Vector3d(mesh->vertices[i * 3], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]));
	}
	glmUnitize(mesh);
	glmFacetNormals(mesh);
	glmVertexNormals(mesh , 90.0);
	FindConnectivity();
	for (int i = 0; i < mesh->numvertices; i++){
		visited.push_back(NON);
		outlier.push_back(false);
		test.push_back(0);
	}

	init();
	
	glutMainLoop();

	return 0;

}