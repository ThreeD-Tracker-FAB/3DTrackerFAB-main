#include "my_gl_texture.h"

const int tex_size = 512;

MyGlTexture::MyGlTexture()
{

}

void MyGlTexture::init()
{
	int i, j;

	for (i = 0; i < tex_size; i++) {
		for (j = 0; j < tex_size; j++) {

			tex_bits[i][j][0] = (GLubyte)0;
			tex_bits[i][j][1] = (GLubyte)50;
			tex_bits[i][j][2] = (GLubyte)0;
		}
	}

	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &tex_id);

	glBindTexture(GL_TEXTURE_2D, tex_id);

	glTexImage2D(
		GL_TEXTURE_2D, 0, GL_RGB, tex_size, tex_size,
		0, GL_RGB, GL_UNSIGNED_BYTE, tex_bits
	);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, 0);
}

void MyGlTexture::update(cv::Mat img)
{
	int i, j;

	cv::Mat img_resized;
	cv::resize(img, img_resized, cv::Size(tex_size, tex_size), 0.0, 0.0, cv::INTER_NEAREST);

	for (i = 0; i < tex_size; i++) {
		for (j = 0; j < tex_size; j++) {

			tex_bits[i][j][0] = (GLubyte)img_resized.data[i * img_resized.step + j * img_resized.elemSize() + 2];
			tex_bits[i][j][1] = (GLubyte)img_resized.data[i * img_resized.step + j * img_resized.elemSize() + 1];
			tex_bits[i][j][2] = (GLubyte)img_resized.data[i * img_resized.step + j * img_resized.elemSize() + 0];
		}
	}

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, tex_id);

	glTexImage2D(
		GL_TEXTURE_2D, 0, GL_RGB, tex_size, tex_size,
		0, GL_RGB, GL_UNSIGNED_BYTE, tex_bits
	);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, 0);
}

GLuint MyGlTexture::getTexId()
{
	return tex_id;
}
