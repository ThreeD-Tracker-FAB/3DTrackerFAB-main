#pragma once

#include <string>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <fstream>

#define MAX_NUM_CAMERA 50

namespace boost {
	namespace serialization {
		// serialization function for Eigen::Matrix
		template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		inline void serialize(
			Archive & ar,
			Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
			const unsigned int file_version
		)
		{
			size_t rows = t.rows(), cols = t.cols();
			ar & BOOST_SERIALIZATION_NVP(rows);
			ar & BOOST_SERIALIZATION_NVP(cols);
			if (rows * cols != t.size())
				t.resize(rows, cols);

			ar & make_nvp("data", make_array(&t(0, 0), t.rows()*t.cols()));
		}
	}
}

struct ROI
{
	Eigen::Vector2f x;
	Eigen::Vector2f y;
	Eigen::Vector2f z;
};

class MyMetadata {
public:

	int num_camera;
	std::string cam_model_name;
	ROI roi;

	std::vector<Eigen::Matrix4f> pc_transforms;
	int ref_cam_id;
	float ref_cam_pos[3];
	float ref_cam_rot[3];
	Eigen::Affine3f ref_cam_transform;

	int calib_color_filt_b_range[2];
	int calib_outlier_removal_meanK;
	float calib_outlier_removal_thresh;
	float calib_pointer_diameter;		// in millimeters

	bool rec_enable_roi_filtering;
	bool rec_enable_voxel_grid_filtering;
	float rec_filter_voxel_size;		// in millimeters

	bool rec_save_2d_vid[MAX_NUM_CAMERA];
	int rec_2d_vid_res[2];

	bool rec_ir_emitter[MAX_NUM_CAMERA];
	int rec_ir_gain;

	MyMetadata()
	{
		calib_color_filt_b_range[0] = 100;
		calib_color_filt_b_range[1] = 255;
		calib_outlier_removal_meanK = 50;
		calib_outlier_removal_thresh = 1.0;
		calib_pointer_diameter = 40.0;

		rec_enable_roi_filtering = false;
		rec_enable_voxel_grid_filtering = false;
		rec_filter_voxel_size = 10.0;

		for (int i = 0; i < MAX_NUM_CAMERA; i++) rec_save_2d_vid[i] = false;

		rec_2d_vid_res[0] = 640;
		rec_2d_vid_res[1] = 480;

		for (int i = 0; i < MAX_NUM_CAMERA; i++) rec_ir_emitter[i] = true;
		rec_ir_gain = 3;
	}

	void updateRefCamTransform()
	{
		Eigen::Affine3f T, t;
		pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, T);
		pcl::getTransformation(0.0f, 0.0f, 0.0f, ref_cam_rot[0] / 180.0*M_PI, 0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI, t);
		T = t * T;
		pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI, ref_cam_rot[2] / 180.0*M_PI, t);
		T = t * T;
		pcl::getTransformation(0.0f, 0.0f, 0.0f, 0.0 / 180.0*M_PI, ref_cam_rot[1] / 180.0*M_PI, 0.0 / 180.0*M_PI, t);
		T = t * T;
		pcl::getTransformation(ref_cam_pos[0], ref_cam_pos[1], ref_cam_pos[2], 0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI, 0.0 / 180.0*M_PI, t);
		T = t * T;

		ref_cam_transform = T;
	}

	int loadFile(const char* filename)
	{
		std::ifstream ifs(filename);
		if (!ifs)
		{
			std::cout << "can not open " << filename << std::endl;
			return 0;
		}

		boost::archive::xml_iarchive ia(ifs);
		ia >> BOOST_SERIALIZATION_NVP(*this);
		ifs.close();

		this->updateRefCamTransform();

		return 1;
	}

	void saveFile(const char* filename)
	{
		std::ofstream ofs(filename);
		boost::archive::xml_oarchive oa(ofs);
		oa << BOOST_SERIALIZATION_NVP(this);
		ofs.close();
	}

	void copyTo(MyMetadata & dst)
	{
		std::stringstream ss;
		boost::archive::xml_oarchive oa(ss);
		oa << BOOST_SERIALIZATION_NVP(this);


		boost::archive::xml_iarchive ia(ss);
		ia >> BOOST_SERIALIZATION_NVP(dst);

		dst.updateRefCamTransform();
	}

private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		if (version >= 1)
		{
			ar & BOOST_SERIALIZATION_NVP(num_camera);
			ar & BOOST_SERIALIZATION_NVP(cam_model_name);
			ar & BOOST_SERIALIZATION_NVP(roi.x);
			ar & BOOST_SERIALIZATION_NVP(roi.y);
			ar & BOOST_SERIALIZATION_NVP(roi.z);
			ar & BOOST_SERIALIZATION_NVP(pc_transforms);
			ar & BOOST_SERIALIZATION_NVP(ref_cam_id);
			ar & BOOST_SERIALIZATION_NVP(ref_cam_pos);
			ar & BOOST_SERIALIZATION_NVP(ref_cam_rot);
		}
		if (version >= 2)
		{
			ar & BOOST_SERIALIZATION_NVP(calib_color_filt_b_range);
			ar & BOOST_SERIALIZATION_NVP(calib_outlier_removal_meanK);
			ar & BOOST_SERIALIZATION_NVP(calib_outlier_removal_thresh);
			ar & BOOST_SERIALIZATION_NVP(calib_pointer_diameter);
			
			ar & BOOST_SERIALIZATION_NVP(rec_enable_roi_filtering);
			ar & BOOST_SERIALIZATION_NVP(rec_enable_voxel_grid_filtering);
			ar & BOOST_SERIALIZATION_NVP(rec_filter_voxel_size);

			ar & BOOST_SERIALIZATION_NVP(rec_save_2d_vid);
			ar & BOOST_SERIALIZATION_NVP(rec_2d_vid_res);

			ar & BOOST_SERIALIZATION_NVP(rec_ir_emitter);
			ar & BOOST_SERIALIZATION_NVP(rec_ir_gain);
		}
	}

};

BOOST_CLASS_VERSION(MyMetadata, 2);
