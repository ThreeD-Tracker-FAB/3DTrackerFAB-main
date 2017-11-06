#pragma once


#include "my_capture.h"

#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


class MyCaptureKinect1 :
	public MyCapture
{
private:
	struct Camera
	{
		INuiSensor* dev;
		HANDLE depth_stream;
		HANDLE color_stream;

		pcl::PointCloud<pcl::PointXYZRGB> pc;
		cv::Mat color_frame;
		cv::Mat depth_frame;
		double timestamp;

		INuiCoordinateMapper* mapper;
	};

public:
	MyCaptureKinect1();
	~MyCaptureKinect1();

	void getNextFrames();
	void getPointClouds();

	void getFrameData(cv::Mat &frame, int camera_id, std::string frame_type);

	void getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id);

	int getNumCamera() { return num_camera; };

	double getFrameTimestamp(int camera_id) { return cameras[camera_id].timestamp; } ;

	void getColorCameraSettings(MyColorCameraSettings & cs, int camera_id);
	void setColorCameraSettings(MyColorCameraSettings & cs, int camera_id);

	bool getInfraredEmitter(int camera_id);
	void setInfraredEmitter(bool emitter_on, int camera_id);

	void setInfraredCamGain(double gain_value);

	void getCameraIntrinsics(INuiCoordinateMapper** cm, int camera_id);

	void setColorFilter(PointCloudFilterSetting pcfs);

	void updateFrame(Camera & c);
	void updatePointCloud(Camera & c);

	static void frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB> & pc,
		INuiCoordinateMapper* intrinsics, const PointCloudFilterSetting & filter_setting);

private:

	int num_camera;

	DWORD timestamp0;

	PointCloudFilterSetting pc_filter_setting;

	std::vector<Camera> cameras;
};

