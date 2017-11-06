#pragma once


#include <opencv2/opencv.hpp>

#include "gl/freeglut.h"

class MyGlTexture
{
public:

	MyGlTexture();

	void init();
	void update(cv::Mat img);

	GLuint getTexId();

private:

	GLuint tex_id;
	GLubyte tex_bits[512][512][3];

};
