//TODO: write comments

// [KNOWN ISSUES]
// - With 60fps in R200, color frame delays

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
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/octree/octree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <shlwapi.h>


///// BUILD OPTIONS	 /////

//#define BUILD_ENABLE_DIO		// comment out if DIO functions are not used.

#ifdef BUILD_ENABLE_DIO

#include "../common/my_dio.h"
MyDIO ttl_dev;
bool frame_ttl_nextsig = false;
bool rec_trigger_by_ttl = false;

#endif

///// END BUILD OPTIONS /////

std::shared_ptr<MyCapture> cap;
std::vector<MyColorCameraSettings> ccs;

MyMetadata metadata;

std::shared_ptr<MyFileIO> fileIO;

bool show_camera_setting_window = false;
bool show_framerate_window = false;
bool show_rec_window = true;
bool show_pc_appearance_setting_window = false;
bool show_camera_monitor = false;

bool color_each_camera = false;
bool recording = false;
bool recording_pause = false;

clock_t rec_t0 = 0;
char rec_session_name[256];

std::vector<bool> disp_camera_pc;

MyFileIO::DataType recording_data_type = MyFileIO::DATA_TYPE_RGBD;

MyGlTexture tex_camera_monitor;
int monitor_camera_id = 0;
bool monitor_depth = false;
cv::Size color_frame_size;
cv::Size color_frame_size_original;

float disp_point_size = 3.0;

int bag_rec_res_i = 0;
int bag_rec_frate_i = 0;


inline float rnd()
{
	return (float)std::rand() / (float)RAND_MAX;
}

void resetIrEmitter()
{
	int i;

	for (i = 0; i < metadata.num_camera; i++)
	{
		cap->setInfraredEmitter(metadata.rec_ir_emitter[i], i);
		metadata.rec_ir_emitter[i] = cap->getInfraredEmitter(i);
	}

	if (metadata.cam_model_name == "D400") cap->setInfraredCamGain(30 + 40 * metadata.rec_ir_gain);
	else cap->setInfraredCamGain(100 + 100 * metadata.rec_ir_gain);
}

void loadColorCameraSettings()
{
	ccs.clear();
	for (int i = 0; i < cap->getNumCamera(); i++)
	{
		MyColorCameraSettings cs;

		cap->getColorCameraSettings(cs, i);

		ccs.push_back(cs);
	}
}

void getProcessedPC(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id)
{
	cap->getPointCloudData(pc, camera_id);

	pcl::transformPointCloud(pc, pc, metadata.pc_transforms[camera_id]);
	pcl::transformPointCloud(pc, pc, metadata.ref_cam_transform);

	if (metadata.rec_enable_roi_filtering)
	{
		pcl::CropBox<pcl::PointXYZRGB> cb;
		cb.setMin(Eigen::Vector4f(metadata.roi.x[0], metadata.roi.y[0], metadata.roi.z[0], 1.0));
		cb.setMax(Eigen::Vector4f(metadata.roi.x[1], metadata.roi.y[1], metadata.roi.z[1], 1.0));
		cb.setInputCloud(pc.makeShared());
		cb.filter(pc);
	}

	if (metadata.rec_enable_voxel_grid_filtering)
	{
		pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> vgf;
		float s = metadata.rec_filter_voxel_size / 1000.0;
		vgf.setLeafSize(s, s, s);
		vgf.setInputCloud(pc.makeShared());
		vgf.filter(pc);
		
	}
}

void startRecording(bool enable_overwrite_warning)
{
	int i;

	metadata.num_camera = cap->getNumCamera();

	char dname[1024];
	sprintf(dname, "data\\%s\\", rec_session_name);

	if (enable_overwrite_warning)
	{
		if (PathFileExistsA(dname))
		{
			ImGui::OpenPopup("Overwrite?");
			return;
		}
	}

	fileIO = std::make_shared<MyFileIO>(dname, rec_session_name, metadata);
	fileIO->saveMetadata();
	
	fileIO->saveCameraIntrinsics(cap);

	recording = true;

	rec_t0 = clock(); 

	if (recording_data_type == MyFileIO::DATA_TYPE_RGBD)
	{
		fileIO->startRgbdWriter();
	}
	else if (recording_data_type == MyFileIO::DATA_TYPE_PC)
	{
		fileIO->startPcWriter();
	}
	else if (recording_data_type == MyFileIO::DATA_TYPE_BAG)
	{
		
		int w, h;

		if (bag_rec_res_i == 0) { w = 424; h = 240;  }
		else if (bag_rec_res_i == 1) { w = 848; h = 480; }
		else if (bag_rec_res_i == 2) { w = 1280; h = 720; }

		/*
		int frate;
		if (bag_rec_frate_i == 0) frate = 30;
		else if (bag_rec_frate_i == 1) frate = 60;
		else if (bag_rec_frate_i == 2) frate = 90;	//only IR
		*/

		std::shared_ptr<MyCaptureD400> cap_d400 = std::static_pointer_cast<MyCaptureD400>(cap);
		cap_d400->startBagRecording(dname, rec_session_name, bag_rec_res_i, bag_rec_frate_i);

		color_frame_size_original = color_frame_size;
		color_frame_size = cv::Size(w, h);
	}

	for (i = 0; i < metadata.num_camera; i++) if (metadata.rec_save_2d_vid[i]) fileIO->start2DVideoWriter(i, metadata.rec_2d_vid_res[0], metadata.rec_2d_vid_res[1]);


#ifdef BUILD_ENABLE_DIO
	frame_ttl_nextsig = true;
	if (rec_trigger_by_ttl) recording_pause = true;
#endif

	ImGui::OpenPopup("Recording...");
}

void stopRecording()
{

#ifdef BUILD_ENABLE_DIO
	ttl_dev.outTTL(false);
#endif

	fileIO->closeFiles();

	if (recording_data_type == MyFileIO::DATA_TYPE_BAG)
	{
		std::shared_ptr<MyCaptureD400> cap_d400 = std::static_pointer_cast<MyCaptureD400>(cap);
		cap_d400->stopBagRecording();

		color_frame_size = color_frame_size_original;
	}

	recording = false;
}

void drawGUI()
{
	int i;

	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplFreeGLUT_NewFrame();

	{
		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{
				if (ImGui::MenuItem("load config", "")) 
				{
					metadata.loadFile("default_setting.xml"); 
					resetIrEmitter();

				}
				if (ImGui::MenuItem("Save config", "")) { metadata.saveFile("default_setting.xml"); }

				ImGui::Separator();

				if (ImGui::MenuItem("Quit", "Alt+F4")) { glutLeaveMainLoop(); }
					
				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Window"))
			{
				if (ImGui::MenuItem("Recording", "", &show_rec_window)) {}

				ImGui::Separator();

				if (ImGui::MenuItem("Camera Setting", "", &show_camera_setting_window))
				{
					if (show_camera_setting_window) loadColorCameraSettings();
				}

				ImGui::Separator();

				if (ImGui::MenuItem("Camera Monitor", "", &show_camera_monitor)) {}
				if (ImGui::MenuItem("Point Cloud Appearance", "", &show_pc_appearance_setting_window)) {}

				ImGui::Separator();

				if (ImGui::MenuItem("Frame rate", "", &show_framerate_window)) {}

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		if (show_framerate_window)
		{
			ImGuiIO& io = ImGui::GetIO();
			ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 130, io.DisplaySize.y - 40));
			ImGui::Begin("Frame Rate Window", nullptr, ImVec2(0, 0), 0.3f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
			ImGui::Text("%.1f frames/sec", io.Framerate);
			ImGui::End();
		}

		if (show_rec_window)
		{
			ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiSetCond_Once);
			ImGui::Begin("Recording Window", &show_rec_window);
			ImGui::Checkbox("ROI filtering", &metadata.rec_enable_roi_filtering);

			ImGui::DragFloatRange2("X (m)", &metadata.roi.x[0], &metadata.roi.x[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
			ImGui::DragFloatRange2("Y (m)", &metadata.roi.y[0], &metadata.roi.y[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
			ImGui::DragFloatRange2("Z (m)", &metadata.roi.z[0], &metadata.roi.z[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f");
			
			ImGui::Separator();

			ImGui::Checkbox("Voxel grid filtering", &metadata.rec_enable_voxel_grid_filtering);
			if (ImGui::SliderFloat("Voxel size (mm)", &metadata.rec_filter_voxel_size, 1.0, 40.0, "%.1f")) {}

			ImGui::Separator();

			ImGui::Text("Save 2D videos: ");

			for (i = 0; i < metadata.num_camera; i++)
			{
				char buf[64];
				sprintf(buf, "%d##save 2d video", i + 1);
				ImGui::SameLine();

				bool chk = metadata.rec_save_2d_vid[i];
				if (ImGui::Checkbox(buf, &chk))
				{
					metadata.rec_save_2d_vid[i] = chk;
				}

			}

			static int sel_res = 0;
			char str_size1[20], str_size2[20], str_size3[20];
			sprintf(str_size1, "%d x %d", color_frame_size.width, color_frame_size.height);
			sprintf(str_size2, "%d x %d", color_frame_size.width*3/4, color_frame_size.height*3/4);
			sprintf(str_size3, "%d x %d", color_frame_size.width/2, color_frame_size.height/2);

			const char* res[] = { str_size1, str_size2, str_size3 };
			if (metadata.rec_2d_vid_res[0] == color_frame_size.width) sel_res = 0;
			else if (metadata.rec_2d_vid_res[0] == color_frame_size.width*3/4) sel_res = 1;
			else if (metadata.rec_2d_vid_res[0] == color_frame_size.width/2) sel_res = 2;
			if (ImGui::Combo("Resolution", &sel_res, res, 3))
			{
				if (sel_res == 0) { metadata.rec_2d_vid_res[0] = color_frame_size.width; metadata.rec_2d_vid_res[1] = color_frame_size.height; }
				if (sel_res == 1) { metadata.rec_2d_vid_res[0] = color_frame_size.width*3/4; metadata.rec_2d_vid_res[1] = color_frame_size.height*3/4; }
				if (sel_res == 2) { metadata.rec_2d_vid_res[0] = color_frame_size.width/2; metadata.rec_2d_vid_res[1] = color_frame_size.height/2; }
			}

			ImGui::Separator();

			ImGui::InputText("Session name", rec_session_name, 256);

			if (!recording)
			{	
				if (ImGui::Button("Start Recording RGBD"))
				{
					recording_data_type = MyFileIO::DATA_TYPE_RGBD;
					startRecording(true);
				}

				if (ImGui::Button("Start Recording Point Cloud"))
				{
					recording_data_type = MyFileIO::DATA_TYPE_PC;
					startRecording(true);
				}

#ifndef BUILD_ENABLE_DIO
				if (metadata.cam_model_name == "D400")
				{
					ImGui::Separator();

					ImGui::Text("Rosbag file recording (only D400)");
					const char* bres[] = { "424 x 240", "848 x 480", "1280 x 720" };
					ImGui::Combo("Resolution##bag", &bag_rec_res_i, bres, 3);
					const char* brate[] = { "30 fps", "60 fps", "90 fps" };
					ImGui::Combo("Frame rate##bag", &bag_rec_frate_i, brate, 3);

					if (ImGui::Button("Start Recording Bag"))
					{
						recording_data_type = MyFileIO::DATA_TYPE_BAG;
						startRecording(true);
					}
				}
#endif
				#ifdef BUILD_ENABLE_DIO
				ImGui::Checkbox("Trigger Recording by TTL", &rec_trigger_by_ttl);
				#endif

				if (ImGui::BeginPopupModal("Overwrite?", NULL, ImGuiWindowFlags_AlwaysAutoResize))
				{
					bool overwrite = false;
					ImGui::Text("The file already exist!");
					if (ImGui::Button("Overwrite", ImVec2(120, 0))) 
					{ 
						ImGui::CloseCurrentPopup(); 
						overwrite = true;
					}
					ImGui::SameLine();
					if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }
					ImGui::EndPopup();

					if (overwrite) startRecording(false);
				}
			}
			else
			{
				if (ImGui::BeginPopupModal("Recording...", NULL, ImGuiWindowFlags_AlwaysAutoResize))
				{
					int w = 500;
					ImGui::Image((void*)(GLuint)tex_camera_monitor.getTexId(), ImVec2(w, w * color_frame_size.height / color_frame_size.width));

					ImGui::Text("Camera:"); ImGui::SameLine();

					for (i = 0; i < cap->getNumCamera(); i++)
					{
						if (i == 0 || i % 4 != 0) ImGui::SameLine();

						char buf[256];
						sprintf(buf, "%d##RecPopup", i + 1);
						ImGui::RadioButton(buf, &monitor_camera_id, i);
					}
					ImGui::SameLine(); ImGui::RadioButton("All##RecPopup", &monitor_camera_id, -1);

					char buf[256];
					#ifdef BUILD_ENABLE_DIO
					if (recording_pause)
					{
						sprintf(buf, "WAITNG FOR TTL INPUT TO START...");
						ImGui::Text(buf);
					}
					else
					{
						sprintf(buf, "Time elapsed:  %7.2f sec", (float)(clock() - rec_t0) / 1000.0);
						ImGui::Text(buf);
					}
					#else
					sprintf(buf, "Time elapsed:  %7.2f sec", (float)(clock() - rec_t0) / 1000.0);
					ImGui::Text(buf);
					#endif

					ImGui::Separator();

					if (ImGui::Button("Stop##Recording", ImVec2(120, 0)))
					{
						ImGui::CloseCurrentPopup();

						stopRecording();
					}

					ImGui::EndPopup();
				}

			}

			ImGui::End();
		}

		if (show_camera_setting_window)
		{
			static int selected_camera = 0;

			ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiSetCond_Once);
			ImGui::Begin("Camera Setting Window", &show_camera_setting_window);

			char buf[256];

			ImGui::Text("IR emitter: ");

			for (i = 0; i < metadata.num_camera; i++)
			{
				char buf[64];
				sprintf(buf, "%d##camera emitter", i + 1);
				ImGui::SameLine();

				bool chk = metadata.rec_ir_emitter[i];
				if (ImGui::Checkbox(buf, &chk))
				{
					cap->setInfraredEmitter(chk, i);
					metadata.rec_ir_emitter[i] = cap->getInfraredEmitter(i);
				}

			}

			ImGui::Separator();

			if (metadata.cam_model_name == "D400")
			{
				ImGui::Text("IR Laser Power: ");
				for (i = 0; i < 4; i++)
				{
					ImGui::SameLine();
					sprintf(buf, "%3d##IR cam gain", 30 + 40 * i);
					if (ImGui::RadioButton(buf, &metadata.rec_ir_gain, i))
					{
						cap->setInfraredCamGain(30 + 40 * i);
					}
				}
			}
			else
			{
				ImGui::Text("IR cam gain: ");
				for (i = 0; i < 4; i++)
				{
					ImGui::SameLine();
					sprintf(buf, "%3d##IR cam gain", 100 + 100 * i);
					if (ImGui::RadioButton(buf, &metadata.rec_ir_gain, i))
					{
						cap->setInfraredCamGain(100 + 100 * i);
					}
				}
			}

			ImGui::End();
		}

		if (show_pc_appearance_setting_window)
		{

			ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiSetCond_Once);
			ImGui::Begin("Point Cloud Appearance Setting Window", &show_pc_appearance_setting_window);

			ImGui::Text("Point cloud preview");

			ImGui::Text("Cameras for disp:");
			for (i = 0; i < disp_camera_pc.size(); i++)
			{
				char buf[64];
				sprintf(buf, "%d##show camera pc", i + 1);
				ImGui::SameLine();

				int ii = (i + 1) % 8;
				float g = ((ii / 4) % 2) * 0.7, b = ((ii / 2) % 2) * 0.7, r = (ii % 2) * 0.7;
				ImColor col(r, g, b);
				ImGui::PushStyleColor(ImGuiCol_CheckMark, (ImVec4)col);

				bool chk = disp_camera_pc[i];
				if (ImGui::Checkbox(buf, &chk)) { disp_camera_pc[i] = chk; }

				ImGui::PopStyleColor();
			}
			
			ImGui::Checkbox("Mark cameras with different colors", &color_each_camera);

			if (ImGui::SliderFloat("Point size", &disp_point_size, 1.0, 20.0, "%.1f")) {}
			

			ImGui::End();
		}
	
		if (show_camera_monitor && !recording)
		{
			ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiSetCond_Once);
			ImGui::Begin("Camera Monitor Window", &show_camera_monitor);

			int w = ImGui::GetWindowWidth() - 15;
			ImGui::Image((void*)(GLuint)tex_camera_monitor.getTexId(), ImVec2(w, w * color_frame_size.height / color_frame_size.width));

			ImGui::Text("Camera:"); ImGui::SameLine();

			for (i = 0; i < cap->getNumCamera(); i++)
			{
				if (i == 0 || i % 4 != 0) ImGui::SameLine();

				char buf[256];
				sprintf(buf, "%d##CameraMonitor", i + 1);
				ImGui::RadioButton(buf, &monitor_camera_id, i);
			}
			ImGui::SameLine(); ImGui::RadioButton("All##CameraMonitor", &monitor_camera_id, -1);

			ImGui::Checkbox("Depth##CameraMonitor", &monitor_depth);

			ImGui::End();
		}

	}
	
    ImGui::Render();
}

void initApp()
{
	metadata.num_camera = cap->getNumCamera();

	metadata.resetCalibrationParams();

	metadata.loadFile("default_setting.xml", metadata.cam_model_name, metadata.num_camera);
	resetIrEmitter();

	disp_camera_pc.clear();
	disp_camera_pc.resize(metadata.num_camera, true);

	sprintf(rec_session_name, "untitled_session");

	tex_camera_monitor.init();

	cv::Mat color_frame;
	cap->getNextFrames();
	cap->getFrameData(color_frame, 0, "COLOR");
	color_frame_size = color_frame.size();

	metadata.rec_2d_vid_res[0] = color_frame_size.width;
	metadata.rec_2d_vid_res[1] = color_frame_size.height;
}

void loopApp()
{

#ifdef BUILD_ENABLE_DIO
	if (rec_trigger_by_ttl)
	{
		if (recording_pause)
		{
			if (ttl_dev.readInput(0))	// check start ttl trigger
			{
				recording_pause = false;
				rec_t0 = clock();
			}
		}
		else if (recording)
		{
			if (ttl_dev.readInput(1))	// check stop ttl trigger
			{
				stopRecording();
			}
		}
	}
	if (recording && !recording_pause)
	{
		ttl_dev.outTTL(frame_ttl_nextsig);
		frame_ttl_nextsig = !frame_ttl_nextsig;
	}
#endif

	cap->getNextFrames();

	if (!recording || recording_data_type == MyFileIO::DATA_TYPE_PC) cap->getPointClouds();

	boost::thread_group thr_grp;

	for (int i = 0; i < cap->getNumCamera(); i++)
	{
		cv::Mat color, depth;
		double ts;

		cap->getFrameData(color, i, "COLOR");
		cap->getFrameData(depth, i, "DEPTH");
		ts = cap->getFrameTimestamp(i);

		if (recording && !recording_pause)
		{
			if (recording_data_type == MyFileIO::DATA_TYPE_RGBD)
			{
				thr_grp.create_thread(boost::bind(&MyFileIO::writeRgbdFrame, fileIO, color, depth, ts, i));
			}
			else if (recording_data_type == MyFileIO::DATA_TYPE_PC)
			{
				pcl::PointCloud<pcl::PointXYZRGB> pc;
				getProcessedPC(pc, i);
				thr_grp.create_thread(boost::bind(&MyFileIO::writePcFrame, fileIO, pc, ts, i));
			}

			if (metadata.rec_save_2d_vid[i]) thr_grp.create_thread(boost::bind(&MyFileIO::write2DVideoFrame, fileIO, color, i));
		}
		
	}

	thr_grp.join_all();
	
	{
		cv::Mat monitor_img;

		if (monitor_camera_id >= 0)
		{
			if (!monitor_depth || recording)
			{
				cap->getFrameData(monitor_img, monitor_camera_id, "COLOR");
			}
			else
			{
				cap->getFrameData(monitor_img, monitor_camera_id, "DEPTH");
				monitor_img.convertTo(monitor_img, CV_8U, 255.0f / 8000.0f, 0.0f);
				cv::cvtColor(monitor_img, monitor_img, CV_GRAY2BGR);
			}
		}
		else
		{
			cv::Mat frame;

			monitor_img = cv::Mat::zeros(color_frame_size, CV_8UC3);

			int n = ceil(sqrt(cap->getNumCamera()));

			for (int i = 0; i < cap->getNumCamera(); i++)
			{
				if (!monitor_depth || recording)
				{
					cap->getFrameData(frame, i, "COLOR");
				}
				else
				{
					cap->getFrameData(frame, i, "DEPTH");
					frame.convertTo(frame, CV_8U, 255.0f / 8000.0f, 0.0f);
					cv::cvtColor(frame, frame, CV_GRAY2BGR);
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
		}
		tex_camera_monitor.update(monitor_img);
	}

	glutPostRedisplay();
}

void displayApp()
{
	glDisable(GL_LIGHTING);

	int i;
	const int n = cap->getNumCamera();

	if (!recording)
	{
		for (i = 0; i < n; i++)
		{
			if (!disp_camera_pc[i]) continue;

			glPointSize(disp_point_size);
			glBegin(GL_POINTS);
			pcl::PointCloud<pcl::PointXYZRGB> pc;
			getProcessedPC(pc, i);

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
		}
	}

	drawROI(metadata.roi);

	drawAxis(0.2);

	drawGUI();

}

int main(int argc, char **argv)
{
	if (checkD400Connection())
	{
		metadata.cam_model_name = "D400";
	}
	else if (checkR200Connection())
	{
		metadata.cam_model_name = "R200";
	}
	else if (checkKinect1Connection())
	{
		metadata.cam_model_name = "Kinect1";
	}
	else
	{
		printf("no camera is connected.\n");
		exit(0);
	}

	StreamMode smode = SMODE_DEPTH_COLOR;
	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-IR") == 0)
		{
			smode = SMODE_DEPTH_IR;
		}
	}
	cap = MyCapture::create(metadata.cam_model_name, { smode, 0, 0, 0 });

	startApp(argc, argv, "Recorder");

    return 0;
}

