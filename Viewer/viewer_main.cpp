
#include "../common/my_capture_kinect1.h"

#include "../common/app_common.h"
#include "../common/my_capture_r200.h"
#include "../common/my_metadata.h"
#include "../common/my_file_io.h"
#include "../common/my_gl_texture.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <iostream>

std::shared_ptr<MyFileIO> fileIO;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_disp;
std::vector<pcl::PointCloud<pcl::Normal>> pc_normal;
std::vector<cv::Mat> color_frames;

MyFileIO::DataType data_type = MyFileIO::DATA_TYPE_RGBD;

std::vector<bool> colorvideo_exist;
MyGlTexture colorvideo_monitor_tex;
int colorvideo_monitor_cam_id = 0;

bool playing = false;

bool enable_speed_regulation = false;

bool color_each_camera = false;
bool enable_roi_filtering = true;
bool show_cam_pos = true;
bool show_axis = true;
float view_pointsize = 3.0;

bool draw_normals = true;

int crnt_frame = 0;

void updateColorVideoFrame()
{
	cv::Mat colorvideo_frame;

	if (data_type == MyFileIO::DATA_TYPE_RGBD)
	{
		colorvideo_monitor_tex.update(color_frames[colorvideo_monitor_cam_id]);
	}
	else
	{
		fileIO->read2DVideoFrame(colorvideo_frame, colorvideo_monitor_cam_id, crnt_frame);
		colorvideo_monitor_tex.update(colorvideo_frame);
	}
}

void mouseAndKey()
{
	ImGuiIO& io = ImGui::GetIO();

	if (ImGui::IsKeyReleased(32))
	{
		playing = !playing;
	}

}

void drawGUI()
{
	int i;

	static bool show_player_window = true;
	static bool show_view_setting_window = false;
	static bool show_colorvideo_monitor = false;
	static bool show_framerate_window = false;

	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplFreeGLUT_NewFrame();

	{
		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{

				ImGui::Separator();

				if (ImGui::MenuItem("Quit", "Alt+F4")) { glutLeaveMainLoop(); }

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Window"))
			{
				if (ImGui::MenuItem("Player", "", &show_player_window)) {}

				if (ImGui::MenuItem("Color Video Monitor", "", &show_colorvideo_monitor)) {}

				if (ImGui::MenuItem("View setting", "", &show_view_setting_window)) {}

				if (ImGui::MenuItem("Frame rate", "", &show_framerate_window)) {}

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

	}
	
	if (show_player_window)
	{ 
		ImGui::SetNextWindowSize(ImVec2(600, 100), ImGuiSetCond_Once);
		ImGui::Begin("Player Window", &show_player_window);

		std::string str = std::to_string((fileIO->getTimestamp(crnt_frame)- fileIO->getTimestamp(0))/1000.0) + "sec###CurrentFrame";
		ImGui::SliderInt(str.c_str(), &crnt_frame, 0, fileIO->getNumFrame() - 1, "Frame: %.0f");

		ImGui::Checkbox("Play", &playing);

		ImGui::SameLine();

		ImGui::Checkbox("Speed regulation", &enable_speed_regulation);

		ImGui::End();
	}

	if (show_colorvideo_monitor)
	{
		ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiSetCond_Once);
		ImGui::Begin("2D Video Monitor", &show_colorvideo_monitor);

		int w = ImGui::GetWindowWidth() - 15;
		ImGui::Image((void*)(GLuint)colorvideo_monitor_tex.getTexId(), ImVec2(w, w * 3 / 4));

		ImGui::Text("Camera:"); ImGui::SameLine();

		for (i = 0; i < fileIO->metadata.num_camera; i++)
		{
			if (i == 0 || i % 4 != 0) ImGui::SameLine();

			if (!colorvideo_exist[i]) continue;
			char buf[256];
			sprintf(buf, "%d##2DVideoMonitor", i + 1);
			if (ImGui::RadioButton(buf, &colorvideo_monitor_cam_id, i)) updateColorVideoFrame();
		}

		if (ImGui::Button("Use the camera view##2DVideoMonitor"))
		{
			pcl::PointCloud<pcl::PointXYZ> pc_camera_pos;
			pcl::PointXYZ p_o(0.0, 0.0, 0.0);
			pcl::PointXYZ p_z(0.0, 0.0, 1.0);
			pcl::PointXYZ p_y(0.0, 1.0, 0.0);
			pc_camera_pos.push_back(p_o);
			pc_camera_pos.push_back(p_z);
			pc_camera_pos.push_back(p_y);

			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.pc_transforms[colorvideo_monitor_cam_id]);
			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.ref_cam_transform);

			std::cout << fileIO->metadata.ref_cam_transform.matrix() << std::endl;

			Eigen::Vector3f v_roi_center(fileIO->metadata.roi.x.mean(), fileIO->metadata.roi.y.mean(), fileIO->metadata.roi.z.mean());
			Eigen::Vector3f v_o(pc_camera_pos[0].x, pc_camera_pos[0].y, pc_camera_pos[0].z);
			Eigen::Vector3f v_z(pc_camera_pos[1].x, pc_camera_pos[1].y, pc_camera_pos[1].z);
			Eigen::Vector3f v_y(pc_camera_pos[2].x, pc_camera_pos[2].y, pc_camera_pos[2].z);

			float d = (v_roi_center - v_o).dot((v_z - v_o).normalized());
			Eigen::Vector3f v_view = v_o + d * (v_z - v_o).normalized();

			Eigen::Matrix4f trans = fileIO->metadata.ref_cam_transform.matrix() * fileIO->metadata.pc_transforms[colorvideo_monitor_cam_id];
			trans = trans.inverse();
			float roll = atan2f(-trans(0, 1), trans(1, 1));

			resetView(v_o.x(), v_o.y(), v_o.z(), v_view.x(), v_view.y(), v_view.z(), roll);

		}

		ImGui::End();
	}

	if (show_view_setting_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("View setting", &show_view_setting_window, ImGuiWindowFlags_AlwaysAutoResize);

		ImGui::Checkbox("ROI filtering##View setting", &enable_roi_filtering);

		ImGui::Checkbox("Show camera position##View setting", &show_cam_pos);

		ImGui::Checkbox("Show Axis##View setting", &show_axis);

		ImGui::Checkbox("Mark cameras with different colors##View setting", &color_each_camera);
		
		ImGui::SliderFloat("Point Size##View setting", &view_pointsize, 1.0, 20.0, "%.1f");

		ImGui::End();
	}

	if (show_framerate_window)
	{
		ImGuiIO& io = ImGui::GetIO();
		ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 130, io.DisplaySize.y - 40));
		ImGui::Begin("Frame Rate Window", nullptr, ImVec2(0, 0), 0.3f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
		ImGui::Text("%.1f frames/sec", io.Framerate);
		ImGui::End();
	}

	mouseAndKey();

    ImGui::Render();
}

void initApp()
{
	int i;

	fileIO = std::make_shared<MyFileIO>("");

	// check data format
	if (fileIO->checkRgbdDataExist()) data_type = MyFileIO::DATA_TYPE_RGBD;
	else if (fileIO->checkPcDataExist()) data_type = MyFileIO::DATA_TYPE_PC;
	else if (fileIO->checkMergedPcDataExist()) data_type = MyFileIO::DATA_TYPE_MERGED_PC;

	// start loading 3D video
	if (data_type == MyFileIO::DATA_TYPE_RGBD) fileIO->startRgbdReader();
	else if (data_type == MyFileIO::DATA_TYPE_PC) fileIO->startPcReader();
	else if (data_type == MyFileIO::DATA_TYPE_MERGED_PC) fileIO->startMergedPcReader();

	pc_disp.resize(fileIO->metadata.num_camera);
	pc_normal.resize(fileIO->metadata.num_camera);

	// for color video monitor
	colorvideo_monitor_tex.init();
	if (data_type == MyFileIO::DATA_TYPE_RGBD)
	{
		color_frames.resize(fileIO->metadata.num_camera);

		colorvideo_exist.resize(fileIO->metadata.num_camera, true);
	}
	else
	{
		fileIO->check2DVideoExist(colorvideo_exist);
		for (i = 0; i < fileIO->metadata.num_camera; i++)
		{
			if (colorvideo_exist[i])
			{
				colorvideo_monitor_cam_id = i;
				break;
			}
		}
		fileIO->start2DVideoReader();
	}

}

void readRGBD(int camera_id, int frame_id)
{
	cv::Mat color_frame, depth_frame;

	fileIO->readRgbdFrame(color_frame, depth_frame, camera_id, frame_id);

	fileIO->rgbdFrame2PointCloud(color_frame, depth_frame, pc_disp[camera_id], camera_id);

	color_frames[camera_id] = color_frame;
}

void loopApp()
{
	static int frame_pre = -1;

	if (playing)
	{
		static clock_t t_pre = clock();

		int next_frame = (crnt_frame + 1) % fileIO->getNumFrame();

		if (!enable_speed_regulation || fileIO->getTimestamp(next_frame) - fileIO->getTimestamp(crnt_frame) <= clock() - t_pre)
		{
			crnt_frame = next_frame;
			t_pre = clock();
		}
	}

	if (frame_pre != crnt_frame)
	{
		int i = 0;

		if (data_type == MyFileIO::DATA_TYPE_RGBD)
		{

			boost::thread_group thr_grp;

			for (i = 0; i < fileIO->metadata.num_camera; i++)
			{
				thr_grp.create_thread(boost::bind(&readRGBD, i, crnt_frame));
			}

			thr_grp.join_all();

		}
		else if (data_type == MyFileIO::DATA_TYPE_PC)
		{
			
			boost::thread_group thr_grp;
			for (i = 0; i < fileIO->metadata.num_camera; i++)
			{
				thr_grp.create_thread(boost::bind(&MyFileIO::readPcFrame, fileIO, boost::ref(pc_disp[i]), i, crnt_frame));
			}
			thr_grp.join_all();
			
		}
		else if (data_type == MyFileIO::DATA_TYPE_MERGED_PC)
		{
			fileIO->readMergedPcFrame(pc_disp[0], pc_normal[0], crnt_frame);
		}

		updateColorVideoFrame();

		frame_pre = crnt_frame;
	}

	glutPostRedisplay();
}

void displayApp()
{
	glDisable(GL_LIGHTING);

	int i;

	for (i = 0; i < fileIO->metadata.num_camera; i++)
	{

		pcl::PointCloud<pcl::PointXYZRGB> pc = pc_disp[i];
			
		if (enable_roi_filtering)
		{
			pcl::CropBox<pcl::PointXYZRGB> cb;
			cb.setMin(Eigen::Vector4f(fileIO->metadata.roi.x[0], fileIO->metadata.roi.y[0], fileIO->metadata.roi.z[0], 1.0));
			cb.setMax(Eigen::Vector4f(fileIO->metadata.roi.x[1], fileIO->metadata.roi.y[1], fileIO->metadata.roi.z[1], 1.0));
			cb.setInputCloud(pc.makeShared());
			cb.filter(pc);
		}

		if (color_each_camera)
		{
			int ii = (i + 1) % 8;
			int g = ((ii / 4) % 2) * 255, b = ((ii / 2) % 2) * 255, r = (ii % 2) * 255;
			glPointSize(view_pointsize);
			glBegin(GL_POINTS);
			for (auto & p : pc)
			{
				glColor3ub(r, g, b);
				glVertex3f(p.x, p.y, p.z);
			}
			glEnd();
		}
		else
		{
			glPointSize(view_pointsize);
			glBegin(GL_POINTS);
			for (auto & p : pc)
			{
				glColor3ub(p.r, p.g, p.b);
				glVertex3f(p.x, p.y, p.z);
			}
			glEnd();

		}

		//draw normals

		if (draw_normals && data_type == MyFileIO::DATA_TYPE_MERGED_PC && i == 0)
		{
			glLineWidth(1.0);
			glBegin(GL_LINES);

			glColor3ub(255, 255, 255);
			
			for (int i_p = 0; i_p < pc_disp[0].size(); i_p++)
			{
				pcl::PointXYZRGB p = pc_disp[0][i_p];
				pcl::Normal n = pc_normal[0][i_p];

				glVertex3f(p.x, p.y, p.z);
				glVertex3f(p.x + n.normal_x*0.01, p.y + n.normal_y*0.01, p.z + n.normal_z*0.01);
			}


			glEnd();
		}

		// show camera position
		
		if (show_cam_pos)
		{
			int ii = (i + 1) % 8;
			int g = ((ii / 4) % 2) * 255, b = ((ii / 2) % 2) * 255, r = (ii % 2) * 255;

			pcl::PointCloud<pcl::PointXYZ> pc_camera_pos;
			pcl::PointXYZ p_o(0.0, 0.0, 0.0);
			pcl::PointXYZ p_z(0.0, 0.0, 0.05);
			pc_camera_pos.push_back(p_o);
			pc_camera_pos.push_back(p_z);

			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.pc_transforms[i]);
			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.ref_cam_transform);

			glPushMatrix();
			glColor3ub(r, g, b);
			glTranslatef(pc_camera_pos[0].x, pc_camera_pos[0].y, pc_camera_pos[0].z);
			glutSolidSphere(0.025, 20, 20);
			glPopMatrix();

			glColor3ub(r, g, b);
			glLineWidth(5.0);
			glBegin(GL_LINES);
			glVertex3f(pc_camera_pos[0].x, pc_camera_pos[0].y, pc_camera_pos[0].z);
			glVertex3f(pc_camera_pos[1].x, pc_camera_pos[1].y, pc_camera_pos[1].z);
			glEnd();

		}
	}
	
	drawROI(fileIO->metadata.roi);

	if (show_axis) drawAxis(0.2);
	
	drawGUI();

}

int main(int argc, char **argv)
{
	startApp(argc, argv, "Viewer");

    return 0;
}

