#include "my_capture.h"
#include "my_capture_d400.h"
#include "my_capture_r200.h"
#include "my_capture_kinect1.h"

#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>


std::shared_ptr<MyCapture> MyCapture::create(const std::string & model_name, StreamSetting ss)
{
	if (model_name == "D400")
	{
		std::cout << "MyCapture::create - D400 is selected as the model" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureD400(ss));
	}
	else if (model_name == "R200")
	{
		std::cout << "MyCapture::create - R200 is selected as the model" << std::endl;
		return std::shared_ptr<MyCapture>(new MyCaptureR200(ss));
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

void preprocessFrame(MyMetadata & metadata, std::vector<pcl::PointCloud<pcl::PointXYZRGB>> & pc_input, pcl::PointCloud<pcl::PointXYZRGBNormal> & pc_merged, float gridsize, std::vector<bool> cam_enable)
{
	int i;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> pc_processed;
	pc_processed.resize(pc_input.size());

	for (i = 0; i < metadata.num_camera; i++)
	{
		if (cam_enable.size() == metadata.num_camera && !cam_enable[i]) continue;

		// ROI filtering
		pcl::CropBox<pcl::PointXYZRGB> cb;
		cb.setMin(Eigen::Vector4f(metadata.roi.x[0], metadata.roi.y[0], metadata.roi.z[0], 1.0));
		cb.setMax(Eigen::Vector4f(metadata.roi.x[1], metadata.roi.y[1], metadata.roi.z[1], 1.0));
		cb.setInputCloud(pc_input[i].makeShared());
		cb.filter(pc_input[i]);

		// VoxelGrid filter
		pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
		vgf.setLeafSize(gridsize, gridsize, gridsize);
		vgf.setInputCloud(pc_input[i].makeShared());
		vgf.filter(pc_input[i]);

		// calculate camera position
		pcl::PointCloud<pcl::PointXYZ> pc_camera_pos;
		pcl::PointXYZ p_o(0.0, 0.0, 0.0);
		pc_camera_pos.push_back(p_o);

		pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, metadata.pc_transforms[i]);
		pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, metadata.ref_cam_transform);

		// calculate surface normal
		pcl::PointCloud<pcl::PointXYZ> pc_input_xyz;

		for (auto p : pc_input[i])
		{
			pcl::PointXYZ pxyz(p.x, p.y, p.z);
			pc_input_xyz.push_back(pxyz);
		}

		pcl::PointCloud<pcl::Normal> pc_n;
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(pc_input_xyz.makeShared());
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(gridsize*5.0);
		ne.setViewPoint(pc_camera_pos[0].x, pc_camera_pos[0].y, pc_camera_pos[0].z);
		ne.compute(pc_n);

		pcl::PointCloud<pcl::PointXYZRGBNormal> pc_to_add;
		pcl::concatenateFields(pc_input[i], pc_n, pc_to_add);

		pc_merged += pc_to_add;
	}

	for (i = 0; i < metadata.num_camera; i++) pc_merged += pc_processed[i];

	// VoxelGrid filter
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> vgf;
	vgf.setLeafSize(gridsize, gridsize, gridsize);
	vgf.setInputCloud(pc_merged.makeShared());
	vgf.filter(pc_merged);
}

bool checkR200Connection()
{
	rs::context rs_ctx;

	if (rs_ctx.get_device_count() > 0) return true;

	return false;
}

bool checkD400Connection()
{
	rs2::context rs_ctx;
	const std::string platform_camera_name = "Platform Camera";

	for (auto&& dev : rs_ctx.query_devices())
	{
		if (dev.get_info(RS2_CAMERA_INFO_NAME) == platform_camera_name) continue;
		else return true;
	}

	return false;
}

bool checkKinect1Connection()
{
	int num_camera = 0;

	::NuiGetSensorCount(&num_camera);

	if (num_camera > 0) return true;

	return false;
}
