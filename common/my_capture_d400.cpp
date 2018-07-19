#include "my_capture_d400.h"

#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <librealsense2/rsutil.h>

MyCaptureD400::MyCaptureD400(StreamSetting ss)
{
	bagrec = false;

	startStreams(ss);

	num_camera = cameras.size();

	std::cout << "number of cameras: " << num_camera << std::endl;

	pc_filter_setting.color_filt_on = false;
	pc_filter_setting.outlier_removal_on = false;
}

void MyCaptureD400::startStreams(StreamSetting ss)
{
	rs2::context rs_ctx;
	const std::string platform_camera_name = "Platform Camera";

	cameras.clear();

	int fps, c_w, c_h, d_w, d_h;

	if (ss.fps == 2) fps = 90;
	else if (ss.fps == 1) fps = 60;
	else fps = 30;

	if (ss.depth_res == 2) { d_w = 1280; d_h = 720; }
	else if (ss.depth_res == 1) { d_w = 848; d_h = 480; }
	else { d_w = 424; d_h = 240; }

	if (ss.color_res == 2) { c_w = 1280; c_h = 720; }
	else if (ss.color_res == 1) { c_w = 848; c_h = 480; }
	else { c_w = 424; c_h = 240; }

	smode = ss.smode;

	for (auto&& dev : rs_ctx.query_devices())
	{
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;

		rs2::pipeline pipe;
		rs2::config cfg;

		cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

		if (smode == SMODE_DEPTH_COLOR)
		{
			cfg.enable_stream(RS2_STREAM_COLOR, c_w, c_h, RS2_FORMAT_BGR8, fps);
			cfg.enable_stream(RS2_STREAM_DEPTH, d_w, d_h, RS2_FORMAT_Z16, fps);
		}
		else if (smode == SMODE_DEPTH_IR)
		{
			cfg.enable_stream(RS2_STREAM_INFRARED, 1, d_w, d_h, RS2_FORMAT_Y8, fps);
			cfg.enable_stream(RS2_STREAM_DEPTH, d_w, d_h, RS2_FORMAT_Z16, fps);
		}

		if (bagrec)
		{
			cfg.enable_record_to_file(bagrec_path + bagrec_sessionname + ".rosbag." + std::to_string(cameras.size() + 1) + ".bag");
		}

		rs2::pipeline_profile prof = pipe.start(cfg);

		Camera cam;

		cam.pipe = pipe;
		cam.pc.clear(); 
		cam.ci.depth_intrin = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

		if (smode == SMODE_DEPTH_COLOR)
		{
			cam.ci.color_intrin = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
			cam.ci.depth_to_color = prof.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(prof.get_stream(RS2_STREAM_COLOR));
			cam.ci.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
		}
		else if (smode == SMODE_DEPTH_IR)
		{
			cam.ci.color_intrin = prof.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>().get_intrinsics();
			cam.ci.depth_to_color = prof.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(prof.get_stream(RS2_STREAM_INFRARED, 1));
			cam.ci.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
		}

		cameras.push_back(cam);
	}
}

void MyCaptureD400::stopStreams()
{
	for (auto & c : cameras)
	{
		c.pipe.stop();
	}
}

void MyCaptureD400::getNextFrames()
{
	boost::thread_group thr_grp;

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureD400::updateFrame, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureD400::updateFrame(Camera & c)
{
	rs2::frameset fset = c.pipe.wait_for_frames();

	rs2::frame color_frame, depth_frame;

	if (smode == SMODE_DEPTH_COLOR)
	{
		color_frame = fset.get_color_frame(); 
		c.color_frame = cv::Mat(cv::Size(c.ci.color_intrin.width, c.ci.color_intrin.height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
	}
	else if (smode == SMODE_DEPTH_IR)
	{
		color_frame = fset.get_infrared_frame();
		cv::Mat ir = cv::Mat(cv::Size(c.ci.color_intrin.width, c.ci.color_intrin.height), CV_8UC1, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::cvtColor(ir, c.color_frame, CV_GRAY2BGR);
	}
	depth_frame = fset.get_depth_frame();
	c.depth_frame = cv::Mat(cv::Size(c.ci.depth_intrin.width, c.ci.depth_intrin.height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
	
	c.timestamp = fset.get_timestamp();
}

void MyCaptureD400::getFrameData(cv::Mat & frame, int camera_id, std::string frame_type)
{
	if (frame_type == "DEPTH")
	{
		frame = cameras[camera_id].depth_frame.clone();
	}
	else if (frame_type == "COLOR")
	{
		frame = cameras[camera_id].color_frame.clone();
	}
}

void MyCaptureD400::getPointClouds()
{
	boost::thread_group thr_grp;

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureD400::updatePointCloud, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureD400::updatePointCloud(Camera & c)
{
	frame2PointCloud(c.color_frame, c.depth_frame, c.pc, c.ci, pc_filter_setting);
}

void MyCaptureD400::frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB>& pc, const RsCameraIntrinsics2 & intrinsics, const PointCloudFilterSetting & filter_setting)
{
	cv::Mat hsv_frame, detection_frame;
	if (filter_setting.color_filt_on) // pre processing for color filtering; finding pixels within the range in HSV color space
	{
		cv::cvtColor(color_frame, hsv_frame, cv::COLOR_BGR2HSV);
		cv::inRange(hsv_frame, filter_setting.color_filt_hsv_min, filter_setting.color_filt_hsv_max, detection_frame);
	}

	pc.clear();

	for (int dy = 0; dy < intrinsics.depth_intrin.height; ++dy)
	{
		for (int dx = 0; dx < intrinsics.depth_intrin.width; ++dx)
		{
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = depth_frame.at<unsigned __int16>(dy, dx);

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			if (depth_value == 0) continue;

			float depth_in_meters = (float)depth_value * intrinsics.scale;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			float depth_pixel[2] = { (float)dx - 0.5f, (float)dy - 0.5f };
			float depth_point[3], color_point[3], color_pixel[2];
			rs2_deproject_pixel_to_point(depth_point, &intrinsics.depth_intrin, depth_pixel, depth_in_meters);
			rs2_transform_point_to_point(color_point, &intrinsics.depth_to_color, depth_point);
			rs2_project_point_to_pixel(color_pixel, &intrinsics.color_intrin, color_point);

			const int cx = static_cast<int>(color_pixel[0] + 0.5f), cy = static_cast<int>(color_pixel[1] + 0.5f);

			if (cx < 0 || cy < 0 || cx >= intrinsics.color_intrin.width || cy >= intrinsics.color_intrin.height) continue;

			if (filter_setting.color_filt_on && detection_frame.at<unsigned char>(cy, cx) == 0) continue;

			pcl::PointXYZRGB p;

			// Use the color from the nearest color pixel
			auto clr = color_frame.at<cv::Vec3b>(cy, cx);
			p.r = clr[2]; p.g = clr[1]; p.b = clr[0];

			// Emit a vertex at the 3D location of this depth pixel
			p.x = -depth_point[0];
			p.y = -depth_point[1];
			p.z = depth_point[2];

			pc.push_back(p);
		}
	}

	if (filter_setting.outlier_removal_on)
	{
		removeNoiseFromThresholdedPc(pc, filter_setting.outlier_filt_meanK, filter_setting.outlier_filt_thresh);
	}
}

void MyCaptureD400::startBagRecording(const std::string & path_data_dir, const std::string & name_session, int res_idx, int fps)
{
	StreamSetting ss = { smode, res_idx, res_idx, fps };
	bagrec = true;
	bagrec_path = path_data_dir;
	bagrec_sessionname = name_session;

	restartStreams(ss);

	/*
	for (int i = 0; i < num_camera; i++)
	{
		std::string s_num = cameras[i].pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		
		cameras[i].pipe.stop();

		rs2::config cfg;

		cfg.enable_device(s_num);
		cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, frate);
		cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, frate);
		
		cfg.enable_record_to_file(path_data_dir + name_session + ".rosbag." + std::to_string(i+1) + ".bag");

		cameras[i].pipe.start(cfg);

		//update intrinsics
		auto prof = cameras[i].pipe.get_active_profile();
		cameras[i].ci.color_intrin = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
		
			cameras[i].ci.depth_intrin = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
			cameras[i].ci.depth_to_color = prof.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(prof.get_stream(RS2_STREAM_COLOR));
			cameras[i].ci.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
	}
	*/
}

void MyCaptureD400::stopBagRecording()
{
	StreamSetting ss = { smode, 0, 0, 30 };
	bagrec = false;

	restartStreams(ss);

	/*
	for (int i = 0; i < num_camera; i++)
	{
		std::string s_num = cameras[i].pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

		cameras[i].pipe.stop();

		rs2::config cfg;

		cfg.enable_device(s_num);
		//!! resolution should be original value !!
		cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 30);
		if (!depth_off) cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 30);

		cameras[i].pipe.start(cfg);

		//update intrinsics
		auto prof = cameras[i].pipe.get_active_profile();
		cameras[i].ci.color_intrin = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
		if (!depth_off)
		{
			cameras[i].ci.depth_intrin = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
			cameras[i].ci.depth_to_color = prof.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(prof.get_stream(RS2_STREAM_COLOR));
			cameras[i].ci.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
		}
	}
	*/
}

void MyCaptureD400::getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB>& pc, int camera_id)
{
	pc = cameras[camera_id].pc;
}

void MyCaptureD400::getCameraIntrinsics(RsCameraIntrinsics2 & ci, int camera_id)
{
	ci = cameras[camera_id].ci;
}

void MyCaptureD400::getColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
	/*
	Camera & c = cameras[camera_id];

	c.dev->get_option_range(rs::option::color_gain, cs.gain.min, cs.gain.max, cs.gain.step);
	c.dev->get_option_range(rs::option::color_exposure, cs.exposure.min, cs.exposure.max, cs.exposure.step);
	c.dev->get_option_range(rs::option::color_contrast, cs.contrast.min, cs.contrast.max, cs.contrast.step);
	c.dev->get_option_range(rs::option::color_brightness, cs.brightness.min, cs.brightness.max, cs.brightness.step);
	c.dev->get_option_range(rs::option::color_gamma, cs.gamma.min, cs.gamma.max, cs.gamma.step);

	cs.gain.value = c.dev->get_option(rs::option::color_gain);
	cs.exposure.value = c.dev->get_option(rs::option::color_exposure);
	cs.contrast.value = c.dev->get_option(rs::option::color_contrast);
	cs.brightness.value = c.dev->get_option(rs::option::color_brightness);
	cs.gamma.value = c.dev->get_option(rs::option::color_gamma);

	cs.auto_exposure = static_cast<bool>(c.dev->get_option(rs::option::color_enable_auto_exposure));
	*/
}

void MyCaptureD400::setColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
	/*
	Camera & c = cameras[camera_id];

	bool flag_update_exposure = (cs.exposure.value != c.dev->get_option(rs::option::color_exposure));

	c.dev->set_option(rs::option::color_gain, cs.gain.value);
	c.dev->set_option(rs::option::color_contrast, cs.contrast.value);
	c.dev->set_option(rs::option::color_brightness, cs.brightness.value);
	c.dev->set_option(rs::option::color_gamma, cs.gamma.value);

	if (flag_update_exposure) {
		c.dev->set_option(rs::option::color_exposure, cs.exposure.value);
		cs.auto_exposure = false;
	}

	bool flag_update_auto_exposure = (cs.auto_exposure != static_cast<bool>(c.dev->get_option(rs::option::color_enable_auto_exposure)));
	if (flag_update_auto_exposure)
	{
		c.dev->set_option(rs::option::color_enable_auto_exposure, static_cast<bool>(cs.auto_exposure));
	}
	//reload color camera setting after setting (some parameters are rounded by R200 after setting)
	getColorCameraSettings(cs, camera_id);
	*/
}

bool MyCaptureD400::getInfraredEmitter(int camera_id)
{
	return cameras[camera_id].pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_option(RS2_OPTION_EMITTER_ENABLED) > 0;
}

void MyCaptureD400::setInfraredEmitter(bool emitter_on, int camera_id)
{
	if (emitter_on)
	{
		cameras[camera_id].pipe.get_active_profile().get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f);
	}
	else
	{
		cameras[camera_id].pipe.get_active_profile().get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f);
	}
}

void MyCaptureD400::setInfraredCamGain(double gain_value)
{
	for (auto c : cameras) c.pipe.get_active_profile().get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_LASER_POWER, gain_value);
}

void MyCaptureD400::setInfraredCamExposure(double exp_value)
{
	
	for (auto c : cameras)
	{
		auto dev = c.pipe.get_active_profile().get_device().first<rs2::depth_sensor>();

		if (exp_value < 0.0)
		{
			dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
		}
		else
		{
			auto range = dev.get_option_range(RS2_OPTION_EXPOSURE);
			dev.set_option(RS2_OPTION_EXPOSURE, exp_value*(range.max - range.min) + range.min);
		}
	}
}

void MyCaptureD400::setColorFilter(PointCloudFilterSetting pcfs)
{
	pc_filter_setting = pcfs;
}

MyCaptureD400::~MyCaptureD400()
{
}

