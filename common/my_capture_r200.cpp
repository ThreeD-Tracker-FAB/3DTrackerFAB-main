#include "my_capture_r200.h"

#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

MyCaptureR200::MyCaptureR200(StreamSetting ss)
{
	int i;

	num_camera = rs_ctx.get_device_count();

	std::cout << "number of cameras: " << num_camera << std::endl;

	startStreams(ss);

	pc_filter_setting.color_filt_on = false;
	pc_filter_setting.outlier_removal_on = false;
}

void MyCaptureR200::startStreams(StreamSetting ss)
{
	int i;

	cameras.clear();
	for (i = 0; i < num_camera; i++)
	{
		Camera c;

		c.dev = rs_ctx.get_device(i);
		c.pc.clear();

		cameras.push_back(c);
	}

	for (auto & c : cameras)
	{
		int fps;
		if (ss.fps == 1) fps = 60;
		else fps = 30;

		{
			int w, h;
			if (ss.depth_res == 1) { w = 480; h = 360; }
			else { w = 320; h = 240; }
			c.dev->enable_stream(rs::stream::depth, w, h, rs::format::z16, fps);
		}

		if (ss.smode == SMODE_DEPTH_COLOR)
		{
			int w, h;
			if (ss.color_res == 0) { w = 640; h = 480; }
			else { w = 640; h = 480; }
			c.dev->enable_stream(rs::stream::color, w, h, rs::format::bgr8, fps);
			c.dev->disable_stream(rs::stream::infrared);
		}
		else if (ss.smode == SMODE_DEPTH_IR)
		{
			int w, h;
			if (ss.depth_res == 1) { w = 480; h = 360; }
			else { w = 320; h = 240; }
			c.dev->enable_stream(rs::stream::infrared, w, h, rs::format::y8, fps);
			c.dev->disable_stream(rs::stream::color);
		}

		smode = ss.smode;

		c.dev->start();
	}
}

void MyCaptureR200::stopStreams()
{
	for (auto & c : cameras)
	{
		c.dev->stop();
	}
}

void MyCaptureR200::getNextFrames()
{
	boost::thread_group thr_grp;

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureR200::updateFrame, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureR200::updateFrame(Camera & c)
{
	rs::intrinsics depth_intrin;
	depth_intrin = c.dev->get_stream_intrinsics(rs::stream::depth);

	rs::intrinsics color_intrin;
	if (smode == SMODE_DEPTH_COLOR) color_intrin = c.dev->get_stream_intrinsics(rs::stream::color);
	else if (smode == SMODE_DEPTH_IR) color_intrin = c.dev->get_stream_intrinsics(rs::stream::infrared);

	c.dev->wait_for_frames();
	c.timestamp = c.dev->get_frame_timestamp(rs::stream::depth);
	if (smode == SMODE_DEPTH_COLOR)
	{
		c.color_frame = cv::Mat(cv::Size(color_intrin.width, color_intrin.height), CV_8UC3, (void*)c.dev->get_frame_data(rs::stream::color), cv::Mat::AUTO_STEP);
	}
	else if (smode == SMODE_DEPTH_IR)
	{
		cv::Mat ir = cv::Mat(cv::Size(color_intrin.width, color_intrin.height), CV_8UC1, (void*)c.dev->get_frame_data(rs::stream::infrared), cv::Mat::AUTO_STEP);
		cv::cvtColor(ir, c.color_frame, CV_GRAY2BGR);
	}
	c.depth_frame = cv::Mat(cv::Size(depth_intrin.width, depth_intrin.height), CV_16UC1, (void*)c.dev->get_frame_data(rs::stream::depth), cv::Mat::AUTO_STEP);
}

void MyCaptureR200::getFrameData(cv::Mat & frame, int camera_id, std::string frame_type)
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

void MyCaptureR200::getPointClouds()
{
	boost::thread_group thr_grp;

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureR200::updatePointCloud, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureR200::updatePointCloud(Camera & c)
{
	RsCameraIntrinsics intrinsics;

	intrinsics.depth_intrin = c.dev->get_stream_intrinsics(rs::stream::depth);
	if (smode == SMODE_DEPTH_COLOR)
	{
		intrinsics.depth_to_color = c.dev->get_extrinsics(rs::stream::depth, rs::stream::color);
		intrinsics.color_intrin = c.dev->get_stream_intrinsics(rs::stream::color);
	}
	else if (smode == SMODE_DEPTH_IR)
	{
		intrinsics.depth_to_color = c.dev->get_extrinsics(rs::stream::depth, rs::stream::infrared);
		intrinsics.color_intrin = c.dev->get_stream_intrinsics(rs::stream::infrared);
	}
	intrinsics.scale = c.dev->get_depth_scale();

	frame2PointCloud(c.color_frame, c.depth_frame, c.pc, intrinsics, pc_filter_setting);
}

void MyCaptureR200::frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB>& pc, const RsCameraIntrinsics & intrinsics, const PointCloudFilterSetting & filter_setting)
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

			float depth_in_meters = depth_value * intrinsics.scale;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depth_pixel = { (float)dx, (float)dy };
			rs::float3 depth_point = intrinsics.depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = intrinsics.depth_to_color.transform(depth_point);
			rs::float2 color_pixel = intrinsics.color_intrin.project(color_point);


			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);

			if (cx < 0 || cy < 0 || cx >= intrinsics.color_intrin.width || cy >= intrinsics.color_intrin.height) continue;

			if (filter_setting.color_filt_on && detection_frame.at<unsigned char>(cy, cx) == 0) continue;

			pcl::PointXYZRGB p;

			// Use the color from the nearest color pixel
			auto clr = color_frame.at<cv::Vec3b>(cy, cx);
			p.r = clr[2]; p.g = clr[1]; p.b = clr[0];

			// Emit a vertex at the 3D location of this depth pixel
			p.x = -depth_point.x;
			p.y = -depth_point.y;
			p.z = depth_point.z;

			pc.push_back(p);
		}
	}

	if (filter_setting.outlier_removal_on)
	{
		removeNoiseFromThresholdedPc(pc, filter_setting.outlier_filt_meanK, filter_setting.outlier_filt_thresh);
	}
}

void MyCaptureR200::getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB>& pc, int camera_id)
{
	pc = cameras[camera_id].pc;
}

void MyCaptureR200::getCameraIntrinsics(RsCameraIntrinsics & ci, int camera_id)
{
	ci.depth_intrin = cameras[camera_id].dev->get_stream_intrinsics(rs::stream::depth);

	if (smode == SMODE_DEPTH_COLOR)
	{
		ci.color_intrin = cameras[camera_id].dev->get_stream_intrinsics(rs::stream::color);
		ci.depth_to_color = cameras[camera_id].dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	}
	else if (smode == SMODE_DEPTH_IR)
	{
		ci.color_intrin = cameras[camera_id].dev->get_stream_intrinsics(rs::stream::infrared);
		ci.depth_to_color = cameras[camera_id].dev->get_extrinsics(rs::stream::depth, rs::stream::infrared);
	}

	ci.scale = cameras[camera_id].dev->get_depth_scale();
}

void MyCaptureR200::getColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
	Camera & c = cameras[camera_id];

	c.dev->get_option_range(rs::option::color_gain, cs.gain.min, cs.gain.max, cs.gain.step);
	c.dev->get_option_range(rs::option::color_exposure, cs.exposure.min, cs.exposure.max, cs.exposure.step);
	c.dev->get_option_range(rs::option::color_contrast , cs.contrast.min, cs.contrast.max, cs.contrast.step);
	c.dev->get_option_range(rs::option::color_brightness, cs.brightness.min, cs.brightness.max, cs.brightness.step);
	c.dev->get_option_range(rs::option::color_gamma, cs.gamma.min, cs.gamma.max, cs.gamma.step);

	cs.gain.value = c.dev->get_option(rs::option::color_gain);
	cs.exposure.value = c.dev->get_option(rs::option::color_exposure);
	cs.contrast.value = c.dev->get_option(rs::option::color_contrast);
	cs.brightness.value = c.dev->get_option(rs::option::color_brightness);
	cs.gamma.value = c.dev->get_option(rs::option::color_gamma);

	cs.auto_exposure = static_cast<bool>(c.dev->get_option(rs::option::color_enable_auto_exposure));
}

void MyCaptureR200::setColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
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
}

bool MyCaptureR200::getInfraredEmitter(int camera_id)
{
	Camera & c = cameras[camera_id];
	
	return static_cast<bool>(c.dev->get_option(rs::option::r200_emitter_enabled));
}

void MyCaptureR200::setInfraredEmitter(bool emitter_on, int camera_id)
{
	Camera & c = cameras[camera_id];

	c.dev->set_option(rs::option::r200_emitter_enabled, emitter_on);
}

void MyCaptureR200::setInfraredCamGain(double gain_value)
{
	for (auto c : cameras) c.dev->set_option(rs::option::r200_lr_gain, gain_value);
}

void MyCaptureR200::setInfraredCamExposure(double exp_value)
{
	for (auto c : cameras)
	{
		double e_min, e_max, e_step;
		c.dev->get_option_range(rs::option::r200_lr_exposure, e_min, e_max, e_step);
		c.dev->set_option(rs::option::r200_lr_exposure, exp_value*(e_max-e_min) + e_min);
	}
}

void MyCaptureR200::setColorFilter(PointCloudFilterSetting pcfs)
{
	pc_filter_setting = pcfs;
}

MyCaptureR200::~MyCaptureR200()
{
}

