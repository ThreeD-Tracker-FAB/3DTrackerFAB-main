#pragma once

// app_common.cpp and app_common.h provide general application framework (3d point cloud view + imGUI)

#include "my_metadata.h"

#include <opencv2/opencv.hpp>

#include "gl/freeglut.h"

#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_freeglut.h"
#include "../imgui/imgui_impl_opengl2.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


void drawAxis(double length);

void drawROI(ROI roi);

void startApp(int argc, char **argv, const char* win_title);

void setView(bool sel, int mx, int my);		//maybe used also externally for mouse picking

void resetView(float x1, float y1, float z1, float x2, float y2, float z2, float roll);		// change view to the specified one (e.g., camera position)

void initApp();	//implement app specific initialization

void loopApp();	//implement app specific loop function

void displayApp();	//implement app specific graphic display function



