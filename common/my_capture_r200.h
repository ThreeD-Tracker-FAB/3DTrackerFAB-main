#pragma once

#include "my_capture.h"
#include <librealsense/rs.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


struct RsCameraIntrinsics
{
	rs::intrinsics depth_intrin;
	rs::extrinsics depth_to_color;
	rs::intrinsics color_intrin;
	float scale;
};

class MyCaptureR200 :
	public MyCapture
{
private:
	struct Camera
	{
		rs::device *dev;
		pcl::PointCloud<pcl::PointXYZRGB> pc;
		cv::Mat color_frame;
		cv::Mat depth_frame;
		double timestamp;
	};

public:

	MyCaptureR200(int c_w, int c_h, bool disable_depth = false);
	~MyCaptureR200();

	void getNextFrames();
	void getFrameData(cv::Mat &frame, int camera_id, std::string frame_type);
	void getPointClouds();
	void getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id);

	int getNumCamera() { return num_camera; };

	double getFrameTimestamp(int camera_id) { return cameras[camera_id].timestamp; };

	void getCameraIntrinsics(RsCameraIntrinsics & ci, int camera_id);

	void getColorCameraSettings(MyColorCameraSettings & cs, int camera_id);
	void setColorCameraSettings(MyColorCameraSettings & cs, int camera_id);

	bool getInfraredEmitter(int camera_id);
	void setInfraredEmitter(bool emitter_on, int camera_id);

	void setInfraredCamGain(double gain_value);

	void setColorFilter(PointCloudFilterSetting pcfs);

	void updateFrame(Camera & c);
	void updatePointCloud(Camera & c);

	static void frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB> & pc,
								 const RsCameraIntrinsics & intrinsics, const PointCloudFilterSetting & filter_setting);


private:

	rs::context rs_ctx;

	int num_camera;

	PointCloudFilterSetting pc_filter_setting;

	std::vector<Camera> cameras;

	bool depth_off;

};

