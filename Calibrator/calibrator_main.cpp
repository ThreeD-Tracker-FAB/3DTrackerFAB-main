
#include "../common/my_capture_kinect1.h"

#include "../common/app_common.h"
#include "../common/my_capture_d400.h"
#include "../common/my_capture_r200.h"
#include "../common/my_metadata.h"
#include "../common/my_file_io.h"
#include "../common/my_gl_texture.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/octree/octree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <shlwapi.h>


std::shared_ptr<MyCapture> cap;
StreamSetting stream_setting;
MyMetadata metadata;

MyGlTexture tex_camera_monitor;
bool monitor_depth = false;
float cam_aspect_ratio;

float disp_point_size = 3.0;
bool color_each_camera = false;
int view_mode = 0;	// 0 = 3D view; 1 = 2D view

int calib_step = 0;
const int max_step = 8;

std::vector<pcl::PointCloud<pcl::PointXYZ>> calibration_trace;
std::vector<pcl::PointCloud<pcl::PointXYZ>> calibration_trace_filtered;

// pointer tracking parameters
float ptp_exposure = 0.1;
int ptp_brightness_thresh = 100;
int ptp_olr_meank = 50;
float ptp_olr_thresh = 0.1;

// trace filtering parameters
static int trace_outlier_removal_meanK = 50;
static float trace_outlier_removal_thresh = 1.0;
static int temp_triming_begin_i = 0;
static int temp_triming_end_i = 1;

bool show_out_roi = false;

void searchPointerCenter(pcl::PointCloud<pcl::PointXYZRGB> &pc, pcl::PointXYZ &p_center)
{
	// get closest point on the pointer and calculate the center of pointer from the value

	int i;

	// algorithm1. simple closest point from camera
	p_center.z = 10000.0;
	for (auto p : pc)
	{
		if (p.z < p_center.z) 
		{
			p_center.x = p.x;
			p_center.y = p.y;
			p_center.z = p.z;
		}
	}

	//p_center.z += metadata.calib_pointer_diameter / 1000.0 / 2.0;

}

void addCalibrationTrace()
{
	int i;

	bool flag_pointer_detected = true;
	for (i = 0; i < cap->getNumCamera(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB> pc;

		cap->getPointCloudData(pc, i);

		if (pc.size() == 0) flag_pointer_detected = false;
	}

	if (flag_pointer_detected)
	{
		for (i = 0; i < cap->getNumCamera(); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB> pc;
			cap->getPointCloudData(pc, i);

			pcl::PointXYZ p_center;
			searchPointerCenter(pc, p_center);

			calibration_trace[i].push_back(p_center);
		}

	}
}

void updateCalibrationTraceFiltered(int meanK, float thresh, int begin_i, int end_i)
{
	int i, j;

	// temporal triming
	std::vector<pcl::PointCloud<pcl::PointXYZ>> calibration_trace_trimed;
	calibration_trace_trimed.clear();
	calibration_trace_trimed.resize(cap->getNumCamera());
	for (i = 0; i < cap->getNumCamera(); i++)
	{
		if (end_i < begin_i) end_i = calibration_trace[i].size();

		for (j = 0; j < calibration_trace[i].size(); j++)
		{
			if (j >= begin_i && j <= end_i) calibration_trace_trimed[i].push_back(calibration_trace[i][j]);
		}
	}

	// outlier removal filtering
	calibration_trace_filtered.clear();
	calibration_trace_filtered.resize(cap->getNumCamera());

	if (calibration_trace_trimed[0].size() < 5) return;		//don't calculate if the number is too small

	std::vector<int> cnt_good(calibration_trace_trimed[0].size(), 0);
	for (i = 0; i < cap->getNumCamera(); i++)
	{
		std::vector<int> ids;

		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(calibration_trace_trimed[i].makeShared());
		sor.setMeanK(meanK);
		sor.setStddevMulThresh(thresh);
		sor.filter(ids);

		for (auto p_i : ids) cnt_good[p_i]++;
	}

	pcl::IndicesPtr ids_good(new std::vector <int>);
	for (i = 0; i < cnt_good.size(); i++)
	{
		if (cnt_good[i] == cap->getNumCamera()) ids_good->push_back(i);
	}

	for (i = 0; i < cap->getNumCamera(); i++)
	{
		pcl::ExtractIndices<pcl::PointXYZ> eifilter;
		eifilter.setInputCloud(calibration_trace_trimed[i].makeShared());
		eifilter.setIndices(ids_good);
		eifilter.filter(calibration_trace_filtered[i]);
	}
}

void updateCalibrationTraceFiltered()
{
	updateCalibrationTraceFiltered(trace_outlier_removal_meanK, trace_outlier_removal_thresh, temp_triming_begin_i, temp_triming_end_i);
}

void updatePcsf(bool tracking)
{
	PointCloudFilterSetting pcfs;

	if (tracking)
	{
		pcfs.color_filt_on = true;
		pcfs.color_filt_hsv_min = cv::Scalar(0, 0, ptp_brightness_thresh);
		pcfs.color_filt_hsv_max = cv::Scalar(255, 255, 255);

		pcfs.outlier_removal_on = false;
		//pcfs.outlier_filt_meanK = ptp_olr_meank;
		//pcfs.outlier_filt_thresh = ptp_olr_thresh;
	}
	else
	{
		pcfs.color_filt_on = false;
		pcfs.outlier_removal_on = false;
	}
	cap->setColorFilter(pcfs);
}

void switchStep(int step, int dir)
{
	if (step == 0)
	{
		view_mode = 0;
		updatePcsf(false);
	}
	else if (step == 1)
	{
		view_mode = 1;
		if (dir == -1)
		{
			StreamSetting ss = { SMODE_DEPTH_COLOR, 0, 0, 0 };
			cap->restartStreams(ss);
			cap->setInfraredCamExposure(-1.0);
			for (int i_cam = 0; i_cam < cap->getNumCamera(); i_cam++) cap->setInfraredEmitter(1, i_cam);
		}
		updatePcsf(false);
	}
	else if (step == 2)
	{
		view_mode = 1;

		if (dir == 1)
		{
			StreamSetting ss = { SMODE_DEPTH_IR, 1, 1, 0 };
			cap->restartStreams(ss);
			for (int i_cam = 0; i_cam < cap->getNumCamera(); i_cam++) cap->setInfraredEmitter(0, i_cam);
		}
		updatePcsf(false);
	}
	else if (step == 3)
	{
		view_mode = 0;
		updatePcsf(true);
	}
	else if (step == 4)
	{
		view_mode = 0;
		updatePcsf(true);

		calibration_trace.clear();
		calibration_trace.resize(cap->getNumCamera());
		for (int i = 0; i < cap->getNumCamera(); i++) calibration_trace[i].clear();

	}
	else if (step == 5)
	{
		view_mode = 0;
		updatePcsf(true);
		if (dir == 1)
		{
			temp_triming_end_i = calibration_trace[0].size();
		}
		updateCalibrationTraceFiltered();
	}
	else if (step == 6)
	{
		view_mode = 0;
		updatePcsf(false);
		if (dir == 1)
		{
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;

			int ref_id = metadata.ref_cam_id;
			for (int i = 0; i < cap->getNumCamera(); i++)
			{
				if (i == ref_id) metadata.pc_transforms[i] = Eigen::Matrix4f::Identity();
				else te.estimateRigidTransformation(calibration_trace_filtered[i], calibration_trace_filtered[ref_id], metadata.pc_transforms[i]);
			}
			StreamSetting ss = { SMODE_DEPTH_COLOR, 0, 0, 0 };
			cap->setInfraredCamExposure(-1.0);
			for (int i_cam = 0; i_cam < cap->getNumCamera(); i_cam++) cap->setInfraredEmitter(1, i_cam);
			cap->restartStreams(ss);
		}
	}
	else if (step == 7)
	{
		view_mode = 0;
		updatePcsf(false);
	}
}

void commonWizardGUI()
{
	ImGui::SameLine(ImGui::GetWindowWidth() - 110);
	ImGui::BeginChild("right pane", ImVec2(100, 0), true, ImGuiWindowFlags_NoScrollbar);
	static bool show_help = false;
	ImGui::Checkbox("Help", &show_help);

	if (calib_step > 0)
	{
		if (ImGui::Button("Prev"))
		{
			calib_step--;
			switchStep(calib_step, -1);
		}
		ImGui::SameLine();
	}
	else
	{
		ImGui::SetCursorPosX(50.0);
	}

	if (calib_step < max_step && ImGui::Button("Next"))
	{
		calib_step++;
		switchStep(calib_step, 1);
	}

	ImGui::EndChild();

}

void getTransformedPC(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id)
{
	cap->getPointCloudData(pc, camera_id);

	if (calib_step != 4) pcl::transformPointCloud(pc, pc, metadata.pc_transforms[camera_id]);
	pcl::transformPointCloud(pc, pc, metadata.ref_cam_transform);

	if (calib_step >= 7 && !show_out_roi)
	{
		pcl::CropBox<pcl::PointXYZRGB> cb;
		cb.setMin(Eigen::Vector4f(metadata.roi.x[0], metadata.roi.y[0], metadata.roi.z[0], 1.0));
		cb.setMax(Eigen::Vector4f(metadata.roi.x[1], metadata.roi.y[1], metadata.roi.z[1], 1.0));
		cb.setInputCloud(pc.makeShared());
		cb.filter(pc);
	}

}

void drawGUI()
{
	int i;

	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplFreeGLUT_NewFrame();

	ImGui::SetNextWindowPos(ImVec2(0, ImGui::GetIO().DisplaySize.y-100));
	ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, 100));
	if (calib_step == 0)
	{
		ImGui::Begin("Step 0: Start page", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		ImGui::Text("Welcome to Calibrator app. This is a software wizard to calibrate multiple depth cameras.");
		if (ImGui::Button("Reset Calibration")) metadata.resetCalibrationParams();
		ImGui::EndChild();
	}
	else if (calib_step == 1)
	{
		ImGui::Begin("Step 1: Configure camera positions", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		ImGui::Text("Move the cameras to cover the space that you want to capture.");
		ImGui::Text("Reference Camera ID:"); ImGui::SameLine();
		for (i = 0; i < cap->getNumCamera(); i++)
		{
			char buf[256];
			sprintf(buf, "%d##refcam_setting", i + 1);

			int ii = (i + 1) % 8;
			float g = ((ii / 4) % 2) * 0.7, b = ((ii / 2) % 2) * 0.7, r = (ii % 2) * 0.7;
			ImColor col(r, g, b);

			ImGui::PushStyleColor(ImGuiCol_CheckMark, (ImVec4)col);

			ImGui::SameLine();
			ImGui::RadioButton(buf, &metadata.ref_cam_id, i);

			ImGui::PopStyleColor();
		}
		ImGui::EndChild();
	}
	else if (calib_step == 2)
	{
		ImGui::Begin("Step 2: Settings exposure for pointer tracking", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		if (ImGui::SliderFloat("Exposure", &ptp_exposure, 0.0, 1.0, "%.4f")) cap->setInfraredCamExposure(ptp_exposure);
		ImGui::EndChild();
	}
	else if (calib_step == 3)
	{
		ImGui::Begin("Step 3: Settings filters for pointer tracking", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		if (ImGui::SliderInt("Brightness threshold", &ptp_brightness_thresh, 0, 255, "%3d")) updatePcsf(true);
		/*
		ImGui::PushItemWidth(200);
		ImGui::Text("Outlier removal filter: ");
		ImGui::SameLine();
		if (ImGui::SliderInt("MeanK", &ptp_olr_meank, 1, 200, "%3d")) updatePcsf(true);
		ImGui::SameLine();
		if (ImGui::SliderFloat("Threshold", &ptp_olr_thresh, 0.001, 5.0, "%.3f", 3.0)) updatePcsf(true);
		*/
		ImGui::EndChild();
	}
	else if (calib_step == 4)
	{
		ImGui::Begin("Step 4: Draw a trace with the pointer", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		ImGui::EndChild();
	}
	else if (calib_step == 5)
	{
		ImGui::Begin("Step 5: Filter the trace", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		ImGui::Text("Temporal triming: ");
		ImGui::SameLine();
		if (ImGui::DragIntRange2("range##Temporal triming - pointer trace", &temp_triming_begin_i, &temp_triming_end_i, 1, 0, calibration_trace[0].size(), "begin: %.0f", "end: %.0f")) updateCalibrationTraceFiltered();
		ImGui::Text("Outlier Removal: ");
		ImGui::PushItemWidth(200);
		ImGui::SameLine();
		if (ImGui::SliderInt("meanK##Outlier Removal - pointer trace", &trace_outlier_removal_meanK, 1, 200)) updateCalibrationTraceFiltered();
		ImGui::SameLine();
		if (ImGui::SliderFloat("Threshold (std dev)##Outlier Removal - pointer trace", &trace_outlier_removal_thresh, 0.001, 5.0, "%.3f", 3.0)) updateCalibrationTraceFiltered();
		ImGui::EndChild();
	}
	else if (calib_step == 6)
	{
		ImGui::Begin("Step 6: Set reference camera position and pose", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		if (ImGui::DragFloat3("Pos (m; XYZ)", metadata.ref_cam_pos, 0.005, -5.0, 5.0, "%.3f")) metadata.updateRefCamTransform();
		if (ImGui::DragFloat3("Rot (deg; XYZ)", metadata.ref_cam_rot, 0.5, -360.0, 360.0, "%.1f"))metadata.updateRefCamTransform();
		ImGui::EndChild();
	}
	else if (calib_step == 7)
	{
		ImGui::Begin("Step 7: ROI setting", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		ImGui::Checkbox("show points outside of ROI", &show_out_roi);

		ImGui::PushItemWidth(200);
		ImGui::DragFloatRange2("X (m)", &metadata.roi.x[0], &metadata.roi.x[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
		ImGui::SameLine();
		ImGui::SetCursorPosX(280.0);
		ImGui::DragFloatRange2("Y (m)", &metadata.roi.y[0], &metadata.roi.y[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
		ImGui::SameLine();
		ImGui::SetCursorPosX(560.0);
		ImGui::DragFloatRange2("Z (m)", &metadata.roi.z[0], &metadata.roi.z[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
		
		ImGui::EndChild();
	}
	else if (calib_step == 8)
	{
		ImGui::Begin("Step 8: Done!", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		ImGui::BeginChild("left pane", ImVec2(0, 0), false);
		if (ImGui::Button("Save Congig and Quit"))
		{
			metadata.saveFile("default_setting.xml");
			glutLeaveMainLoop();
		}
		ImGui::EndChild();
	}
	else
	{
		ImGui::Begin("Unknown step...?", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
	}

	commonWizardGUI();
	
	ImGui::End();

	if (view_mode == 0) // 3D view
	{
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2(0, 0));
		ImGui::Begin("3D View", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		if (ImGui::Button("Switch to 2D")) view_mode = 1;
		ImGui::SameLine();
		ImGui::Checkbox("Mark cameras with different colors", &color_each_camera);
		ImGui::End();
	}
	else if (view_mode == 1) // 2D view
	{
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y - 100));
		ImGui::Begin("2D View", 0, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
		if (ImGui::Button("Switch to 3D")) view_mode = 0;
		ImGui::SameLine();
		ImGui::Checkbox("Depth", &monitor_depth);

		float w = ImGui::GetWindowWidth() - 15;
		float h = ImGui::GetWindowHeight() - 70;
		if (w * cam_aspect_ratio > h) ImGui::Image((void*)(GLuint)tex_camera_monitor.getTexId(), ImVec2(h / cam_aspect_ratio, h));
		else ImGui::Image((void*)(GLuint)tex_camera_monitor.getTexId(), ImVec2(w, w * cam_aspect_ratio));

		ImGui::End();
	}

	ImGui::Render();
}

void initApp()
{
	cap = MyCapture::create(metadata.cam_model_name);

	cap->setInfraredCamExposure(-1.0);
	for (int i_cam = 0; i_cam < cap->getNumCamera(); i_cam++) cap->setInfraredEmitter(1, i_cam);

	metadata.num_camera = cap->getNumCamera();
	metadata.resetCalibrationParams();

	metadata.loadFile("default_setting.xml", metadata.cam_model_name, metadata.num_camera);

	tex_camera_monitor.init();
}

void loopApp()
{
	cap->getNextFrames();
	cap->getPointClouds();

	if (calib_step == 4) addCalibrationTrace();

	if (view_mode == 1)
	{
		cv::Mat monitor_img;
		cv::Mat frame;

		monitor_img = cv::Mat::zeros(cv::Size(640.0, static_cast<int>(640.0 * cam_aspect_ratio)), CV_8UC3);

		int n = ceil(sqrt(cap->getNumCamera()));

		for (int i = 0; i < cap->getNumCamera(); i++)
		{
			if (!monitor_depth)
			{
				cap->getFrameData(frame, i, "COLOR");
			}
			else
			{
				cap->getFrameData(frame, i, "DEPTH");
				float d_min = 0.0;
				float d_max = 2000.0;
				float scale = 255.0 / (d_max - d_min);
				frame.convertTo(frame, CV_8UC1, scale, -d_min * scale);
				cv::Mat mask(frame.rows, frame.cols, CV_8UC1);
				for (int i_r = 0; i_r < frame.rows; i_r++)
				{
					for (int i_c = 0; i_c < frame.cols; i_c++)
					{
						if (frame.at<uchar>(i_r, i_c) == 0)
							mask.at<uchar>(i_r, i_c) = 0;
						else
							mask.at<uchar>(i_r, i_c) = 255;
					}
				}
				cv::bitwise_not(frame, frame);
				cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
				for (int i_r = 0; i_r < frame.rows; i_r++)
				{
					for (int i_c = 0; i_c < frame.cols; i_c++)
					{
						if (mask.at<uchar>(i_r, i_c) == 0)
						{
							frame.at<cv::Vec3b>(i_r, i_c)[0] = 0;
							frame.at<cv::Vec3b>(i_r, i_c)[1] = 0;
							frame.at<cv::Vec3b>(i_r, i_c)[2] = 0;
						}
					}
				}
			}

			std::vector<cv::Point2f> src, dst;

			src.push_back(cv::Point2f(0, 0));
			src.push_back(cv::Point2f(frame.cols, 0));
			src.push_back(cv::Point2f(frame.cols, frame.rows));

			cv::Point2f p0((i % n)*monitor_img.cols / n, (i / n)*monitor_img.rows / n);
			cv::Point2f p1((i % n)*monitor_img.cols / n + monitor_img.cols / n, (i / n)*monitor_img.rows / n + monitor_img.rows / n);

			dst.push_back(p0);
			dst.push_back(cv::Point2f(p1.x, p0.y));
			dst.push_back(p1);

			cv::Mat aff_mat = cv::getAffineTransform(src, dst);

			cv::warpAffine(frame, monitor_img, aff_mat, monitor_img.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
		}
		tex_camera_monitor.update(monitor_img);
	}

	glutPostRedisplay();
	
}

void displayApp()
{
	glDisable(GL_LIGHTING);

	if (view_mode == 0)
	{
		for (int i = 0; i < cap->getNumCamera(); i++)
		{
			//if (!disp_camera_pc[i]) continue;

			glPointSize(disp_point_size);
			glBegin(GL_POINTS);
			pcl::PointCloud<pcl::PointXYZRGB> pc;
			getTransformedPC(pc, i);

			if (color_each_camera)
			{
				int ii = (i + 1) % 8;
				int g = ((ii / 4) % 2) * 255, b = ((ii / 2) % 2) * 255, r = (ii % 2) * 255;
				for (auto & p : pc)
				{
					glColor3ub(r, g, b);
					glVertex3f(p.x, p.y, p.z);
				}
			}
			else
			{
				for (auto & p : pc)
				{
					glColor3ub(p.r, p.g, p.b);
					glVertex3f(p.x, p.y, p.z);
				}

			}
			glEnd();

			if (calib_step == 4 || calib_step == 5)
			{
				glPointSize(3.0);

				pcl::PointCloud<pcl::PointXYZ> pc;
				if (calib_step == 4) pcl::transformPointCloud(calibration_trace[i], pc, metadata.ref_cam_transform);
				else pcl::transformPointCloud(calibration_trace_filtered[i], pc, metadata.ref_cam_transform);

				int ii = (i + 1) % 8;
				int g = ((ii / 4) % 2) * 255, b = ((ii / 2) % 2) * 255, r = (ii % 2) * 255;

				glBegin(GL_POINTS);
				glColor3ub(r, g, b);
				for (auto & p : pc)
				{
					glVertex3f(p.x, p.y, p.z);
				}
				glEnd();

			}
		}

		drawROI(metadata.roi);

		drawAxis(0.2);
	}
	
	drawGUI();
}

int main(int argc, char **argv)
{
	stream_setting.smode = SMODE_DEPTH_IR;
	stream_setting.fps = 0;

	if (checkD400Connection())
	{
		metadata.cam_model_name = "D400";
		stream_setting.color_res = 0;
		stream_setting.depth_res = 0;
		cam_aspect_ratio = 9.0 / 16.0;
	}
	else if (checkR200Connection())
	{
		metadata.cam_model_name = "R200";
		stream_setting.color_res = 0;
		stream_setting.depth_res = 0;
		cam_aspect_ratio = 3.0 / 4.0;
	}
	else if (checkKinect1Connection())
	{
		metadata.cam_model_name = "Kinect1";
		stream_setting.color_res = 0;
		stream_setting.depth_res = 0;
	}
	else
	{
		printf("no camera is connected.\n");
		exit(0);
	}

	startApp(argc, argv, "Calibrator");

    return 0;
}

