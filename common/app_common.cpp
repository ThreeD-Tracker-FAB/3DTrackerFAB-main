
#include "app_common.h"

#include <windows.h>

GLfloat viewZ;
GLfloat mvmat[16];

unsigned int app_screen_width = 960;
unsigned int app_screen_height = 720;

void drawAxis(double length)
{
	GLUquadricObj *arrows[3];

	glLineWidth(3.0);

	// Draw X-axis
	glColor3ub(255, 0, 0);
	glBegin(GL_LINES);
	glVertex3d(-length, 0, 0);
	glVertex3d(length, 0, 0);
	glEnd();
	glPushMatrix();
	arrows[0] = gluNewQuadric();
	gluQuadricDrawStyle(arrows[0], GLU_FILL);
	glTranslated(length, 0.0f, 0.0f);
	glRotated(90.0f, 0, 1, 0);
	gluCylinder(arrows[0], length / 10, 0.0f, length / 2.5, 8, 8);
	glPopMatrix();

	// Draw Y-axis
	glColor3ub(0, 255, 0);
	glBegin(GL_LINES);
	glVertex3d(0, -length, 0);
	glVertex3d(0, length, 0);
	glEnd();
	glPushMatrix();
	arrows[1] = gluNewQuadric();
	gluQuadricDrawStyle(arrows[1], GLU_FILL);
	glTranslated(0.0f, length, 0.0f);
	glRotated(-90.0f, 1, 0, 0);
	gluCylinder(arrows[1], length / 10, 0.0f, length / 2.5, 8, 8);
	glPopMatrix();

	// Draw Z-axis
	glColor3ub(0, 0, 255);
	glBegin(GL_LINES);
	glVertex3d(0, 0, -length);
	glVertex3d(0, 0, length);
	glEnd();
	glPushMatrix();
	arrows[2] = gluNewQuadric();
	gluQuadricDrawStyle(arrows[2], GLU_FILL);
	glTranslated(0.0f, 0.0f, length);
	gluCylinder(arrows[2], length / 10, 0.0f, length / 2.5, 8, 8);
	glPopMatrix();
}

void drawROI(ROI roi)
{
	glLineWidth(2.0);

	Eigen::Vector3f roi_center, roi_size;

	roi_center << roi.x.mean(), roi.y.mean(), roi.z.mean();
	roi_size << roi.x[1] - roi.x[0], roi.y[1] - roi.y[0], roi.z[1] - roi.z[0];

	glDisable(GL_LIGHTING);

	glLineWidth(2.0);
	glColor3ub(100, 100, 100);

	glPushMatrix();
	glTranslatef(roi_center.x(), roi_center.y(), roi_center.z());
	glScalef(roi_size.x(), roi_size.y(), roi_size.z());
	glutWireCube(1.0);
	glPopMatrix();
}

void setView(bool sel, int mx, int my)
{
	int w, h;

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	/* ウィンドウ全体をビューポートにする */
	w = glutGet(GLUT_WINDOW_WIDTH);
	h = glutGet(GLUT_WINDOW_HEIGHT);
	glViewport(0, 0, w, h);

	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);

	/* 透視変換行列の指定 */
	glMatrixMode(GL_PROJECTION);

	/* 透視変換行列の初期化 */
	glLoadIdentity();
	if (sel) gluPickMatrix(mx, vp[3] - my, 1, 1, vp);

	gluPerspective(50.0, (double)w / (double)h, 0.1, 1000.0);

	/* モデルビュー変換行列の指定 */
	glMatrixMode(GL_MODELVIEW);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* モデルビュー変換行列の初期化 */
	glLoadIdentity();

	/* 視点の移動 */
	gluLookAt(0.0, 0.0, viewZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	GLfloat lpos[4] = { 0.0, 0.0, -10.0, 0.0 };

	/* 光源の位置を設定 */
	glLightfv(GL_LIGHT0, GL_POSITION, lpos);

	glMultMatrixf(mvmat);
}

void drawScene()
{
	setView(false, 0, 0);

	//app specific display func
	displayApp();

	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	glutSwapBuffers();
}

/*
bool keyboardEvent(unsigned char nChar, int nX, int nY)
{
	ImGuiIO& io = ImGui::GetIO();

	switch (nChar)
	{
	case 127: // delete
		io.KeysDown[nChar] = true; break;
	case 8: // backspace
		io.KeysDown[nChar] = true; break;
	
	case 1: // Ctrl+A
		io.KeysDown[nChar] = true; break;	//does not work
	case 3: // Ctrl+C
		io.KeysDown[nChar] = true; break;	//does not work
	case 22: // Ctrl+V
		io.KeysDown[nChar] = true; break;	//does not work
	case 24: // Ctrl+X
		io.KeysDown[nChar] = true; break;	//does not work
	case 25: // Ctrl+Y
		io.KeysDown[nChar] = true; break;	//does not work
	case 26: // Ctrl+Z
		io.KeysDown[nChar] = true; break;	//does not work
	
	default:
		io.AddInputCharacter(nChar);
		if (!io.WantCaptureKeyboard) io.KeysDown[nChar] = true;
	}

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
	
	return true;
}

void KeyboardSpecial(int key, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	//io.AddInputCharacter(key);
	io.KeysDown[key] = true;

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

void keyboardUpCallback(unsigned char nChar, int nX, int nY)
{
	ImGuiIO& io = ImGui::GetIO();

	io.KeysDown[nChar] = false;

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

void KeyboardSpecialUp(int key, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.KeysDown[key] = false;

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

bool mouseEvent(int button, int state, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();

	io.MousePos = ImVec2((float)x, (float)y);

	if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON))
		io.MouseDown[0] = true;
	else
		io.MouseDown[0] = false;

	if (state == GLUT_DOWN && (button == GLUT_RIGHT_BUTTON))
		io.MouseDown[1] = true;
	else
		io.MouseDown[1] = false;

	if (state == GLUT_DOWN && (button == GLUT_MIDDLE_BUTTON))
		io.MouseDown[2] = true;
	else
		io.MouseDown[2] = false;

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;

	return true;
}

void mouseWheel(int button, int dir, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);
	if (dir > 0)
	{
		// Zoom in
		io.MouseWheel = 1.0;
	}
	else if (dir < 0)
	{
		// Zoom out
		io.MouseWheel = -1.0;
	}

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

void reshape(int w, int h)
{
	// update screen width and height for imgui new frames
	app_screen_width = w;
	app_screen_height = h;
}

void keyboardCallback(unsigned char nChar, int x, int y)
{
	if (keyboardEvent(nChar, x, y))
	{
		glutPostRedisplay();
	}
}

void mouseCallback(int button, int state, int x, int y)
{
	if (mouseEvent(button, state, x, y))
	{
	}
}

void mouseMoveCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

*/
void mouseDragCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();

	float x_pre = io.MousePos.x;
	float y_pre = io.MousePos.y;

	io.MousePos = ImVec2((float)x, (float)y);

	if (io.MouseDown[1] && !io.WantCaptureMouse)		//right btn
	{
		if (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
		{
			double dx, dy;

			dx = (io.MousePos.x - x_pre) / (float)(io.DisplaySize.x) * 10.0 * fabs(viewZ) / 10.0;
			dy = (io.MousePos.y - y_pre) / (float)(io.DisplaySize.y) * 10.0 * fabs(viewZ) / 10.0;

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(-dx, -dy, 0.0);
			glMultMatrixf(mvmat);
			glGetFloatv(GL_MODELVIEW_MATRIX, mvmat);
			glPopMatrix();
		}
		else if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
		{
			double dx, dy, a;

			dx = (io.MousePos.x - x_pre) / (float)(io.DisplaySize.x) * 2.0;
			dy = (io.MousePos.y - y_pre) / (float)(io.DisplaySize.y) * 2.0;

			a = dx + dy;

			viewZ -= viewZ*a;

		}
		else
		{
			float dx, dy;

			dx = (io.MousePos.x - x_pre) / (float)(io.DisplaySize.x) * 6.28 * 50.0;
			dy = -(io.MousePos.y - y_pre) / (float)(io.DisplaySize.y) * 6.28 * 50.0;

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			glRotatef(dx, 0.0, 1.0, 0.0);
			glRotatef(dy, 1.0, 0.0, 0.0);
			glMultMatrixf(mvmat);
			glGetFloatv(GL_MODELVIEW_MATRIX, mvmat);
			glPopMatrix();
		}
	}

	io.KeyCtrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
	io.KeyShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
}

void mainLoop(void)
{
	//app specific loop func
	loopApp();
}


static void guiSetClipboardText(void* user_data, const char* text)
{
	int    buf_size;
	char  *buf;
	HANDLE h_mem;

	buf_size = strlen(text) + 1;
	h_mem = GlobalAlloc(GMEM_SHARE | GMEM_MOVEABLE, buf_size);
	if (!h_mem) return;

	buf = (char *)GlobalLock(h_mem);
	if (buf)
	{
		strcpy_s(buf, buf_size, text);
		GlobalUnlock(h_mem);
		if (OpenClipboard(NULL))
		{
			EmptyClipboard();                  
			SetClipboardData(CF_TEXT, h_mem);
			CloseClipboard();
		}
	}
}

static const char* guiGetClipboardText(void* user_data)
{
	HANDLE h_mem;
	PTSTR str_clip;

	static std::string text;

	if (OpenClipboard(NULL) && (h_mem = GetClipboardData(CF_TEXT)))
	{
		str_clip = static_cast<PTSTR>(GlobalLock(h_mem));

		text = std::string((char*)str_clip);

		GlobalUnlock(h_mem);
		CloseClipboard();
	}

	return text.c_str();
}


void startApp(int argc, char **argv, const char* win_title)
{
	glutInit(&argc, argv);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);

	glutInitWindowSize(app_screen_width, app_screen_height);
	glutInitWindowPosition(200, 200);
	glutCreateWindow(win_title);

	// callback
	/*
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboardCallback);
	glutSpecialFunc(KeyboardSpecial);
	glutKeyboardUpFunc(keyboardUpCallback);
	glutSpecialUpFunc(KeyboardSpecialUp);
	glutMouseFunc(mouseCallback);
	glutMouseWheelFunc(mouseWheel);
	glutMotionFunc(mouseDragCallback);
	glutPassiveMotionFunc(mouseMoveCallback);
	*/
	glutMotionFunc(mouseDragCallback);
	glutPassiveMotionFunc(mouseDragCallback);
	glutDisplayFunc(drawScene);
	glutIdleFunc(mainLoop);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetFloatv(GL_MODELVIEW_MATRIX, mvmat);
	viewZ = -10.0;

	glClearColor(0.447f, 0.565f, 0.604f, 1.0f);


	// Setup ImGui binding
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

	ImGui_ImplFreeGLUT_Init();
	ImGui_ImplFreeGLUT_InstallFuncs();
	ImGui_ImplOpenGL2_Init();

	//register clipboard functions 
	io.SetClipboardTextFn = guiSetClipboardText;
	io.GetClipboardTextFn = guiGetClipboardText;
	io.ClipboardUserData = NULL;

	// Setup style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsClassic();

	// load font
	io.Fonts->AddFontDefault(); 
	auto roboto = io.Fonts->AddFontFromMemoryCompressedTTF(font_roboto_compressed_data, font_roboto_compressed_size, 16.0f);
	io.FontDefault = roboto;

	//no imgui.ini file saving
	io.IniFilename = NULL;

	//app specific initialization
	initApp();

	glutMainLoop();

	// Cleanup
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplFreeGLUT_Shutdown();
	ImGui::DestroyContext();
}

void resetView(float x1, float y1, float z1, float x2, float y2, float z2, float roll)
{
	float d;

	//look at (x2, y2, z2) from (x1, y1, z1) (in complicated manner for compatibility with manual view changing)
	Eigen::Vector3f v;
	Eigen::Vector3f P1(x1, y1, z1);
	Eigen::Vector3f P2(x2, y2, z2);
	
	v = P2 - P1;
	d = v.norm();
	v.normalize();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetFloatv(GL_MODELVIEW_MATRIX, mvmat);
	viewZ = -d;

	v.normalize();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRotatef(roll / M_PI*180.0, 0.0, 0.0, 1.0);

	glRotatef(asin(v.y()) / M_PI*180.0, 1.0, 0.0, 0.0);
	
	if (v.x() < 0) glRotatef(acos(v.z()/sqrt(v.x()*v.x() + v.z()*v.z()) ) / M_PI*180.0, 0.0, 1.0, 0.0);
	else glRotatef(-acos(v.z() / sqrt(v.x()*v.x() + v.z()*v.z())) / M_PI*180.0, 0.0, 1.0, 0.0);


	glTranslatef(-(x1+v.x()*d), -(y1 + v.y()*d), -(z1 + v.z()*d));

	glGetFloatv(GL_MODELVIEW_MATRIX, mvmat);

	glPopMatrix();
}