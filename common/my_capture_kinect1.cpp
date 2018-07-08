#include "my_capture_kinect1.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>

MyCaptureKinect1::MyCaptureKinect1()
{
	int i;

	::NuiGetSensorCount(&num_camera);

	std::cout << "number of cameras: " << num_camera << std::endl;

	cameras.clear();
	for (i = 0; i < num_camera; i++)
	{
		Camera c;

		::NuiCreateSensorByIndex(i, &c.dev);

		c.pc.clear();

		cameras.push_back(c);
	}

	for (auto & c : cameras)
	{
		c.dev->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
		c.dev->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_320x240, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE, 2, 0, &c.depth_stream);
		c.dev->NuiSetForceInfraredEmitterOff(false);
		c.dev->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &c.color_stream);

		c.dev->NuiGetCoordinateMapper(&c.mapper);

	}

	//wait for kinects ready
	for (auto & c : cameras)
	{
		HANDLE streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
		c.dev->NuiSetFrameEndEvent(streamEvent, 0);
		::WaitForSingleObject(streamEvent, INFINITE);
	}

	::Sleep(10000);	//wait for kinects ready completely

	pc_filter_setting.color_filt_on = false;
	pc_filter_setting.outlier_removal_on = false;

	HANDLE streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	cameras[0].dev->NuiSetFrameEndEvent(streamEvent, 0);
	::WaitForSingleObject(streamEvent, INFINITE);
	::ResetEvent(streamEvent);
	NUI_IMAGE_FRAME depthFrame = { 0 };
	cameras[0].dev->NuiImageStreamGetNextFrame(cameras[0].depth_stream, 0, &depthFrame);
	timestamp0 = depthFrame.liTimeStamp.LowPart;

}

MyCaptureKinect1::~MyCaptureKinect1()
{
	for (auto & c : cameras)
	{
		c.mapper->Release();
		c.dev->NuiShutdown();
		c.dev->Release();
	}
}

void MyCaptureKinect1::startStreams(StreamSetting ss)
{
}

void MyCaptureKinect1::stopStreams()
{
}

void MyCaptureKinect1::getNextFrames()
{
	boost::thread_group thr_grp;

	HANDLE streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	cameras[0].dev->NuiSetFrameEndEvent(streamEvent, 0);
	::WaitForSingleObject(streamEvent, INFINITE);
	::ResetEvent(streamEvent);

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureKinect1::updateFrame, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureKinect1::getPointClouds()
{
	boost::thread_group thr_grp;

	for (auto & c : cameras)
	{
		thr_grp.create_thread(boost::bind(&MyCaptureKinect1::updatePointCloud, this, boost::ref(c)));
	}

	thr_grp.join_all();
}

void MyCaptureKinect1::getFrameData(cv::Mat & frame, int camera_id, std::string frame_type)
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

void MyCaptureKinect1::getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB>& pc, int camera_id)
{
	pc = cameras[camera_id].pc;
}

void MyCaptureKinect1::getColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
	Camera & c = cameras[camera_id];

	INuiColorCameraSettings *nui_cs = 0;
	c.dev->NuiGetColorCameraSettings(&nui_cs);

	BOOL b;
	nui_cs->GetAutoExposure(&b);
	cs.auto_exposure = b;

	nui_cs->GetMinGain(&cs.gain.min);
	nui_cs->GetMinExposureTime(&cs.exposure.min);
	nui_cs->GetMinContrast(&cs.contrast.min);
	nui_cs->GetMinBrightness(&cs.brightness.min);
	nui_cs->GetMinGamma(&cs.gamma.min);

	nui_cs->GetMaxGain(&cs.gain.max);
	nui_cs->GetMaxExposureTime(&cs.exposure.max);
	nui_cs->GetMaxContrast(&cs.contrast.max);
	nui_cs->GetMaxBrightness(&cs.brightness.max);
	nui_cs->GetMaxGamma(&cs.gamma.max);
	
	nui_cs->GetGain(&cs.gain.value);
	nui_cs->GetExposureTime(&cs.exposure.value);
	nui_cs->GetContrast(&cs.contrast.value);
	nui_cs->GetBrightness(&cs.brightness.value);
	nui_cs->GetGamma(&cs.gamma.value);
}

void MyCaptureKinect1::setColorCameraSettings(MyColorCameraSettings & cs, int camera_id)
{
	Camera & c = cameras[camera_id];

	INuiColorCameraSettings *nui_cs = 0;
	c.dev->NuiGetColorCameraSettings(&nui_cs);

	nui_cs->SetAutoExposure(cs.auto_exposure);
	nui_cs->SetGain(cs.gain.value);
	nui_cs->SetExposureTime(cs.exposure.value);
	nui_cs->SetContrast(cs.contrast.value);
	nui_cs->SetBrightness(cs.brightness.value);
	nui_cs->SetGamma(cs.gamma.value);

	//reload color camera setting after setting (some parameters are rounded by R200 after setting)
	getColorCameraSettings(cs, camera_id);
}

bool MyCaptureKinect1::getInfraredEmitter(int camera_id)
{
	// not implemented yet

	return true;
}

void MyCaptureKinect1::setInfraredEmitter(bool emitter_on, int camera_id)
{
	// not implemented yet
}

void MyCaptureKinect1::setInfraredCamGain(double gain_value)
{
	std::cout << "IR cam gain is not implemented for Kinect..." << std::endl;
}

void MyCaptureKinect1::getCameraIntrinsics(INuiCoordinateMapper** cm, int camera_id)
{
	*cm = cameras[camera_id].mapper;
}

void MyCaptureKinect1::setColorFilter(PointCloudFilterSetting pcfs)
{
	pc_filter_setting = pcfs;
}

void MyCaptureKinect1::updateFrame(Camera & c)
{
	NUI_IMAGE_FRAME depthFrame = { 0 };
	c.dev->NuiImageStreamGetNextFrame(c.depth_stream, 0, &depthFrame);
	NUI_LOCKED_RECT depthData = { 0 };
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	c.depth_frame = cv::Mat(240, 320, CV_16UC1, depthData.pBits).clone();

	NUI_IMAGE_FRAME colorFrame = { 0 };
	c.dev->NuiImageStreamGetNextFrame(c.color_stream, 0, &colorFrame);
	NUI_LOCKED_RECT colorData;
	colorFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);
	
	cv::Mat clr_mat = cv::Mat(480, 640, CV_8UC4, colorData.pBits).clone();
	cv::cvtColor(clr_mat, c.color_frame, cv::COLOR_BGRA2BGR);

	c.timestamp = static_cast<double>(depthFrame.liTimeStamp.LowPart - timestamp0);

	c.dev->NuiImageStreamReleaseFrame(c.depth_stream, &depthFrame);
	c.dev->NuiImageStreamReleaseFrame(c.color_stream, &colorFrame);
}

void MyCaptureKinect1::updatePointCloud(Camera & c)
{
	frame2PointCloud(c.color_frame, c.depth_frame, c.pc, c.mapper, pc_filter_setting);
}

void MyCaptureKinect1::frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB>& pc, INuiCoordinateMapper * intrinsics, const PointCloudFilterSetting & filter_setting)
{
	pc.clear();

	cv::Mat hsv_frame, detection_frame;
	if (filter_setting.color_filt_on) // pre processing for color filtering; finding pixels within the range in HSV color space
	{
		cv::cvtColor(color_frame, hsv_frame, cv::COLOR_BGR2HSV);
		cv::inRange(hsv_frame, filter_setting.color_filt_hsv_min, filter_setting.color_filt_hsv_max, detection_frame);
	}

	for (int x = 0; x < 320; x++) for (int y = 0; y < 240; y++)
	{
		USHORT distance = depth_frame.at<USHORT>(y, x);

		if (distance == 0) continue;

		Vector4 real = NuiTransformDepthImageToSkeleton(x, y, distance, NUI_IMAGE_RESOLUTION_320x240);
		pcl::PointXYZRGB point;
		point.x = real.x;
		point.y = real.y;
		point.z = real.z;

		NUI_COLOR_IMAGE_POINT cp;
		intrinsics->MapSkeletonPointToColorPoint(&real, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &cp);

		if (filter_setting.color_filt_on && detection_frame.at<unsigned char>(cp.y, cp.x) == 0) continue;

		if (cp.y >= 480 || cp.x >= 640 || cp.y < 0 || cp.x < 0) continue;
		cv::Vec3b color = color_frame.at<cv::Vec3b>(cp.y, cp.x);
		point.r = color[2];
		point.g = color[1];
		point.b = color[0];

		pc.push_back(point);
	}

	if (filter_setting.outlier_removal_on)
	{
		removeNoiseFromThresholdedPc(pc, filter_setting.outlier_filt_meanK, filter_setting.outlier_filt_thresh);
	}

}
