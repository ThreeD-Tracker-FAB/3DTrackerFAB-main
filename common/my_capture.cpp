#include "my_capture.h"
#include "my_capture_r200.h"
#include "my_capture_kinect1.h"

#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


std::shared_ptr<MyCapture> MyCapture::create(const std::string & model_name)
{
	if (model_name == "R200")
	{
		std::cout << "MyCapture::create - R200 is selected as the model (color:320x240 mode)" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureR200(320,240));
	}
	else if (model_name == "R200_C640x480")
	{
		std::cout << "MyCapture::create - R200 is selected as the model (color:640x480 mode)" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureR200(640, 480));
	}
	else if (model_name == "R200_C1920x1080")
	{
		std::cout << "MyCapture::create - R200 is selected as the model (color:1920x1080 mode)" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureR200(1920, 1080));
	}
	else if (model_name == "Kinect1")
	{
		std::cout << "MyCapture::create - Kinect1 is selected as the model" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureKinect1());
	}

	std::cout << "MyCapture::create - Unknown camera model" << std::endl;
	return std::shared_ptr<MyCapture>(nullptr);
}


void removeNoiseFromThresholdedPc(pcl::PointCloud<pcl::PointXYZRGB> & pc, int meanK, float thresh)
{
	if (pc.size() == 0) return;
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	outrem.setInputCloud(pc.makeShared());
	outrem.setRadiusSearch(thresh);
	outrem.setMinNeighborsInRadius(meanK);
	outrem.filter(pc);
}