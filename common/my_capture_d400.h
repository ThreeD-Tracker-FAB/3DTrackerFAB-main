#pragma once

#pragma once

#include "my_capture.h"
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


struct RsCameraIntrinsics2
{
	rs2_intrinsics depth_intrin;
	rs2_extrinsics depth_to_color;
	rs2_intrinsics color_intrin;
	float scale;
};

class MyCaptureD400 :
	public MyCapture
{
private:
	struct Camera
	{
		rs2::pipeline pipe;

		RsCameraIntrinsics2 ci;

		pcl::PointCloud<pcl::PointXYZRGB> pc;
		cv::Mat color_frame;
		cv::Mat depth_frame;
		double timestamp;
	};

public:

	MyCaptureD400(int c_w, int c_h, bool disable_depth = false);
	~MyCaptureD400();

	void getNextFrames();
	void getFrameData(cv::Mat &frame, int camera_id, std::string frame_type);
	void getPointClouds();
	void getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id);

	int getNumCamera() { return num_camera; };

	double getFrameTimestamp(int camera_id) { return cameras[camera_id].timestamp; };

	void getCameraIntrinsics(RsCameraIntrinsics2 & ci, int camera_id);

	void getColorCameraSettings(MyColorCameraSettings & cs, int camera_id);
	void setColorCameraSettings(MyColorCameraSettings & cs, int camera_id);

	bool getInfraredEmitter(int camera_id);
	void setInfraredEmitter(bool emitter_on, int camera_id);

	void setInfraredCamGain(double gain_value);

	void setColorFilter(PointCloudFilterSetting pcfs);

	void updateFrame(Camera & c);
	void updatePointCloud(Camera & c);

	static void frame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB> & pc,
		const RsCameraIntrinsics2 & intrinsics, const PointCloudFilterSetting & filter_setting);

	void startBagRecording(const std::string & path_data_dir, const std::string & name_session, int w = 848, int h = 480, int frate = 30);
	void stopBagRecording();

private:

	int num_camera;

	PointCloudFilterSetting pc_filter_setting;

	std::vector<Camera> cameras;

	bool depth_off;

};

