#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sstream>

#include "my_metadata.h"

struct SettingParam
{
	double value;
	double min;
	double max;
	double step;
};

struct MyColorCameraSettings
{
	bool auto_exposure;

	SettingParam gain;
	SettingParam exposure;

	SettingParam contrast;
	SettingParam brightness;

	SettingParam gamma;
};

struct PointCloudFilterSetting
{
	bool color_filt_on;
	cv::Scalar color_filt_hsv_min;
	cv::Scalar color_filt_hsv_max;

	bool outlier_removal_on;
	int outlier_filt_meanK;
	float outlier_filt_thresh;
};

// Interface class for 3D video capture
class MyCapture
{
public:
	virtual ~MyCapture() {};

	// acquire the next frames in all the cameras
	virtual void getNextFrames() = 0;	

	// generate pointclouds of all the cameras based on the lastly acquired frames
	virtual void getPointClouds() = 0;

	// get color or depth frame data of specified camera
	virtual void getFrameData(cv::Mat &frame, int camera_id, std::string frame_type) = 0;

	// get point cloud data of specified camera
	virtual void getPointCloudData(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id) = 0;

	// get parameters of the cameras
	virtual int getNumCamera() = 0;

	// get frametimestamp
	virtual double getFrameTimestamp(int camera_id) = 0;

	// get & set color camera settings
	virtual void getColorCameraSettings(MyColorCameraSettings & cs, int camera_id) = 0;
	virtual void setColorCameraSettings(MyColorCameraSettings & cs, int camera_id) = 0;

	// get & set infrared emitter settings 
	virtual bool getInfraredEmitter(int camera_id) = 0;
	virtual void setInfraredEmitter(bool emitter_on, int camera_id) = 0;

	// set infrared camera gain
	virtual void setInfraredCamGain(double gain_value) = 0;

	// set point cloud filtering with color
	virtual void setColorFilter(PointCloudFilterSetting pcfs) = 0;

	// factory for different camera models
	static std::shared_ptr<MyCapture> create(const std::string & model_name);

};

// convenience functions
void removeNoiseFromThresholdedPc(pcl::PointCloud<pcl::PointXYZRGB> & pc, int meanK, float thresh); // removing outlier noise from point cloud
void preprocessFrame(MyMetadata & metadata, std::vector<pcl::PointCloud<pcl::PointXYZRGB>> & pc_input, pcl::PointCloud<pcl::PointXYZRGBNormal> & pc_merged, float gridsize, std::vector<bool> cam_enable = std::vector<bool>());
bool checkR200Connection(); // check R200 camera connections
bool checkD400Connection(); // check D400 camera connections
bool checkKinect1Connection(); // check Kinect1 camera connections
