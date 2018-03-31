#include "my_file_io.h"

#include <iostream>
#include <direct.h>
#include <shlwapi.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <vtk_zlib.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

MyFileIO::MyFileIO() 
	: data_dir(""), session_name("")
{
}

MyFileIO::MyFileIO(const std::string & metadatafilepath)
{
	if (metadatafilepath == "")
	{
		OPENFILENAMEA ofn;
		char szFile[MAX_PATH] = "";
		ZeroMemory(&ofn, sizeof(ofn));
		ofn.lStructSize = sizeof(OPENFILENAMEA);
		ofn.lpstrFilter = "Metadata (*.metadata.xml)\0*.metadata.xml\0";
		ofn.lpstrFile = szFile;
		ofn.nMaxFile = MAX_PATH;
		ofn.Flags = OFN_FILEMUSTEXIST;

		GetOpenFileNameA(&ofn);

		std::string fullpath(ofn.lpstrFile);

		int path_i = fullpath.find_last_of("\\") + 1;

		data_dir = fullpath.substr(0, path_i);

		std::string metadatafilename = fullpath.substr(path_i, fullpath.length());

		int ss_i = metadatafilename.find(".metadata.xml");

		session_name = metadatafilename.substr(0, ss_i);
	}
	else
	{
		int path_i = metadatafilepath.find_last_of("\\") + 1;

		data_dir = metadatafilepath.substr(0, path_i);

		std::string metadatafilename = metadatafilepath.substr(path_i, metadatafilepath.length());

		int ss_i = metadatafilename.find(".metadata.xml");

		session_name = metadatafilename.substr(0, ss_i);

	}

	loadMetadata();

}

MyFileIO::MyFileIO(const std::string & path_data_dir, const std::string & name_session, MyMetadata & md)
	: data_dir(path_data_dir), session_name(name_session)
{
	if (PathIsDirectoryA(data_dir.c_str()) == 0)
	{
		_mkdir(data_dir.c_str());
	}
	md.copyTo(metadata);
}

MyFileIO::~MyFileIO()
{
}

void MyFileIO::saveMetadata()
{
	std::string filepath = data_dir + session_name + ".metadata.xml";
	metadata.saveFile(filepath.c_str());
}

void MyFileIO::loadMetadata()
{
	std::string filepath = data_dir + session_name + ".metadata.xml";

	metadata.loadFile(filepath.c_str());
}

void MyFileIO::saveTrackingResult(RodentTrackerResult & result)
{
	std::string filepath = data_dir + session_name + ".trackresult.bin";
	result.save(filepath.c_str());
}

void MyFileIO::loadTrackingResult(RodentTrackerResult & result)
{
	std::string filepath = data_dir + session_name + ".trackresult.bin";
	result.load(filepath.c_str());
}

void MyFileIO::saveTrackingParam(RodentTrackerParam & param)
{
	std::string filepath = data_dir + session_name + ".trackparam.txt";
	param.save(filepath.c_str());
}

void MyFileIO::loadTrackingParam(RodentTrackerParam & param)
{
	std::string filepath = data_dir + session_name + ".trackparam.txt";
	param.load(filepath.c_str());
}

bool MyFileIO::checkRgbdDataExist()
{
	std::string filepath;
	int i = 0;

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".rgbd.frame." + std::to_string(i + 1) + ".bin";
		if (!PathFileExistsA(filepath.c_str())) return false;
		filepath = data_dir + session_name + ".rgbd.ts." + std::to_string(i + 1) + ".txt";
		if (!PathFileExistsA(filepath.c_str())) return false;
	}

	return true;
}

bool MyFileIO::checkPcDataExist()
{
	std::string filepath;
	int i = 0;

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".pc.frame." + std::to_string(i + 1) + ".bin";
		if (!PathFileExistsA(filepath.c_str())) return false;
		filepath = data_dir + session_name + ".pc.ts." + std::to_string(i + 1) + ".txt";
		if (!PathFileExistsA(filepath.c_str())) return false;
	}

	return true;
}

bool MyFileIO::checkMergedPcDataExist()
{
	std::string filepath;
	filepath = data_dir + session_name + ".mrgpc.frame.bin";
	if (!PathFileExistsA(filepath.c_str())) return false;
	filepath = data_dir + session_name + ".mrgpc.ts.txt";
	if (!PathFileExistsA(filepath.c_str())) return false;

	return true;
}

bool MyFileIO::checkTrackingResultExist()
{
	std::string filepath;
	filepath = data_dir + session_name + ".trackresult.bin";
	if (!PathFileExistsA(filepath.c_str())) return false;

	return true;
}

bool MyFileIO::checkTrackingParamExist()
{
	std::string filepath;
	filepath = data_dir + session_name + ".trackparam.txt";
	if (!PathFileExistsA(filepath.c_str())) return false;

	return true;
}

void MyFileIO::check2DVideoExist(std::vector<bool> & vid_exist)
{
	vid_exist.clear();
	vid_exist.resize(metadata.num_camera);

	std::string filepath;
	int i = 0;

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".2dvideo." + std::to_string(i + 1) + ".avi";
		vid_exist[i] = PathFileExistsA(filepath.c_str());
	}
}

void MyFileIO::saveCameraIntrinsics(std::shared_ptr<MyCapture> & cap)
{
	if (metadata.cam_model_name == "R200" || metadata.cam_model_name == "R200_C640x480" || metadata.cam_model_name == "R200_C1920x1080")
		saveCameraIntrinsics(std::static_pointer_cast<MyCaptureR200>(cap));
	else if (metadata.cam_model_name == "Kinect1")
		saveCameraIntrinsics(std::static_pointer_cast<MyCaptureKinect1>(cap));
}

void MyFileIO::saveCameraIntrinsics(std::shared_ptr<MyCaptureR200> & cap)
{
	std::string filepath;

	for (int i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".camintrin." + std::to_string(i + 1) + ".bin";
		FILE *fp = fopen(filepath.c_str(), "wb");

		RsCameraIntrinsics ci;
		cap->getCameraIntrinsics(ci, i);

		saveCameraIntrinsics(ci, fp);

		fclose(fp);

	}
}

void MyFileIO::saveCameraIntrinsics(std::shared_ptr<MyCaptureKinect1> & cap)
{
	std::string filepath;

	for (int i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".camintrin." + std::to_string(i + 1) + ".bin";
		FILE *fp = fopen(filepath.c_str(), "wb");

		INuiCoordinateMapper * cm = nullptr;
		cap->getCameraIntrinsics(&cm, i);

		saveCameraIntrinsics(cm, fp);

		fclose(fp);

	}
}

void MyFileIO::startRgbdWriter()
{
	closeFiles();
	
	std::string filepath;
	int i = 0;

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".rgbd.frame." + std::to_string(i + 1) + ".bin";
		std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
		ofs_framedata.push_back(std::move(ofs));
	}
	
	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".rgbd.ts." + std::to_string(i + 1) + ".txt";
		std::ofstream ofs(filepath, std::ios::out);
		ofs_timestamp.push_back(std::move(ofs));
	}
}

void MyFileIO::writeRgbdFrame(cv::Mat color_frame, cv::Mat depth_frame, double timestamp, int camera_id)
{
	fpos_t pos = ofs_framedata[camera_id].tellp().seekpos();
	ofs_timestamp[camera_id] << std::fixed << timestamp << "   " << pos << std::endl;

	saveRgbdFrame(color_frame, depth_frame, ofs_framedata[camera_id]);

}

void MyFileIO::startPcWriter()
{
	closeFiles();

	std::string filepath;
	int i = 0;

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".pc.frame." + std::to_string(i + 1) + ".bin";
		std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
		ofs_framedata.push_back(std::move(ofs));
	}

	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".pc.ts." + std::to_string(i + 1) + ".txt";
		std::ofstream ofs(filepath, std::ios::out);
		ofs_timestamp.push_back(std::move(ofs));
	}
}

void MyFileIO::writePcFrame(pcl::PointCloud<pcl::PointXYZRGB>& pc, double timestamp, int camera_id)
{
	bool showStatistics = false;

	fpos_t pos = ofs_framedata[camera_id].tellp().seekpos();
	ofs_timestamp[camera_id] << std::fixed << timestamp << "   " << pos << std::endl;

	encodePc(pc, ofs_framedata[camera_id]);

}

void MyFileIO::start2DVideoWriter(int camera_id, int w, int h, int fourcc)
{
	if (cv_videowriters.size() < metadata.num_camera) cv_videowriters.resize(metadata.num_camera);
	cv_videowriters_size = cv::Size(w, h);

	std::string filepath;
	filepath = data_dir + session_name + ".2dvideo." + std::to_string(camera_id+1) + ".avi";

	cv_videowriters[camera_id] = std::shared_ptr<cv::VideoWriter>(new cv::VideoWriter(filepath, fourcc, 30.0, cv_videowriters_size));
}

void MyFileIO::write2DVideoFrame(cv::Mat color_frame, int camera_id)
{
	if (!cv_videowriters[camera_id]) return;

	if (color_frame.size().width == cv_videowriters_size.width && color_frame.size().height == cv_videowriters_size.height)
	{
		*cv_videowriters[camera_id] << color_frame;
	}
	else
	{
		cv::Mat resized_frame;
		cv::resize(color_frame, resized_frame, cv_videowriters_size, 0.0, 0.0, cv::INTER_NEAREST);

		*cv_videowriters[camera_id] << resized_frame;
	}
}

void MyFileIO::preprocessFrame(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> & pc_input, pcl::PointCloud<pcl::PointXYZRGBNormal> & pc_merged, float gridsize, std::vector<bool> cam_enable)
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

void MyFileIO::preprosessData(float gridsize, std::vector<bool> cam_enable)
{
	int i;
	DataType data_type = DATA_TYPE_NONE;

	clock_t t0 = clock();

	if (checkRgbdDataExist()) data_type = DATA_TYPE_RGBD;
	if (checkPcDataExist()) data_type = DATA_TYPE_PC;

	if (data_type == DATA_TYPE_NONE)
	{
		std::cout << "No data files for pre-processing..." << std::endl;
	}

	if (data_type == DATA_TYPE_RGBD) startRgbdReader();
	if (data_type == DATA_TYPE_PC) startPcReader();

	size_t num_frame = 0;

	for (auto fi : frame_index) if (fi.size() > num_frame) num_frame = fi.size();

	std::string filepath;

	{
		filepath = data_dir + session_name + ".mrgpc.frame.bin";
		std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
		ofs_framedata.push_back(std::move(ofs));
	}

	{
		filepath = data_dir + session_name + ".mrgpc.ts.txt";
		std::ofstream ofs(filepath, std::ios::out);
		ofs_timestamp.push_back(std::move(ofs));
	}

	for (int i_frame = 0; i_frame < num_frame; i_frame++)
	{

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pc_input;
		pc_input.resize(metadata.num_camera);

		if (data_type == DATA_TYPE_RGBD)
		{
			for (i = 0; i < metadata.num_camera; i++)
			{
				if (cam_enable.size() == metadata.num_camera && !cam_enable[i]) continue;
				cv::Mat color_frame, depth_frame;
				readRgbdFrame(color_frame, depth_frame, i, i_frame);
				rgbdFrame2PointCloud(color_frame, depth_frame, pc_input[i], i);
			}
		}

		if (data_type == DATA_TYPE_PC)
		{
			for (i = 0; i < metadata.num_camera; i++)
			{
				if (cam_enable.size() == metadata.num_camera && !cam_enable[i]) continue;
				readPcFrame(pc_input[i], i, i_frame);
			}
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal> pc_merged;

		preprocessFrame(pc_input, pc_merged, gridsize, cam_enable);
		
		// write frame timestamp to file
		fpos_t pos = ofs_framedata[0].tellp().seekpos();
		ofs_timestamp[0] << std::fixed << frame_index[0][i_frame].ts << "   " << pos << std::endl;

		// write point cloud to file

		pcl::PointCloud<pcl::PointXYZRGB> pc_out_xyzrgb;
		pcl::PointCloud<pcl::Normal> pc_out_n;

		for (auto p : pc_merged)
		{
			pcl::PointXYZRGB pp;
			pcl::Normal pn;

			pp.x = p.x; pp.y = p.y; pp.z = p.z;
			pp.r = p.r; pp.g = p.g; pp.b = p.b;

			pc_out_xyzrgb.push_back(pp);

			pn.normal_x = p.normal_x;
			pn.normal_y = p.normal_y;
			pn.normal_z = p.normal_z;

			pc_out_n.push_back(pn);
		}

		if (pc_out_xyzrgb.size() != pc_out_n.size()) std::cout << "!!!" << std::endl;

		encodePc(pc_out_xyzrgb, pc_out_n, ofs_framedata[0]);

		if (num_frame > 50 && (i_frame + 1) % (size_t)(num_frame / 50) == 0) std::cout << "#";
	}
	std::cout << " - finished" << std::endl;

	closeFiles();

	//std::cout << clock() - t0 << " ms took for preprocessing" << std::endl;
}

void MyFileIO::startRgbdReader()
{
	closeFiles();

	int i;
	std::string filepath;

	// make frame index from timestamp data  -- !!code repetition!!

	for (auto fi : frame_index) fi.clear();
	frame_index.clear();

	frame_index.resize(metadata.num_camera);
	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".rgbd.ts." + std::to_string(i + 1) + ".txt";
		std::ifstream ifs(filepath, std::ios::in);

		FrameIndex fi;

		while (!ifs.eof())
		{
			ifs >> fi.ts >> fi.fpos;

			frame_index[i].push_back(fi);
		}

	}

	// open input frame data
	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".rgbd.frame." + std::to_string(i + 1) + ".bin";
		std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
		ifs_framedata.push_back(std::move(ifs));
	}

	// load cameraintrinsics
	camera_intrinsics.resize(metadata.num_camera);
	camera_intrinsics_k.resize(metadata.num_camera);

	for (i = 0; i < metadata.num_camera; i++)
	{

		if (metadata.cam_model_name == "R200" || metadata.cam_model_name == "R200_C640x480" || metadata.cam_model_name == "R200_C1920x1080")
		{
			std::string filepath = getDataPathHeader() + ".camintrin." + std::to_string(i + 1) + ".bin";
			FILE *fp = fopen(filepath.c_str(), "rb");
			loadCameraIntrinsics(camera_intrinsics[i], fp);
			fclose(fp);
		}
		else if (metadata.cam_model_name == "Kinect1")
		{
			std::string filepath = getDataPathHeader() + ".camintrin." + std::to_string(i + 1) + ".bin";
			FILE *fp = fopen(filepath.c_str(), "rb");
			loadCameraIntrinsics(&camera_intrinsics_k[i], fp);
			fclose(fp);
		}
	}

}

void MyFileIO::readRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, int camera_id, size_t frame_id)
{
	ifs_framedata[camera_id].seekg(frame_index[camera_id][frame_id].fpos, std::ios_base::beg);

	loadRgbdFrame(color_frame, depth_frame, ifs_framedata[camera_id]);
}

void MyFileIO::rgbdFrame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame, pcl::PointCloud<pcl::PointXYZRGB>& pc, const int camera_id)
{
	PointCloudFilterSetting fsetting;

	fsetting.color_filt_on = false;
	fsetting.outlier_removal_on = false;

	if (metadata.cam_model_name == "R200" || metadata.cam_model_name == "R200_C640x480" || metadata.cam_model_name == "R200_C1920x1080")
	{
		MyCaptureR200::frame2PointCloud(color_frame, depth_frame, pc, camera_intrinsics[camera_id], fsetting);
	}
	else if (metadata.cam_model_name == "Kinect1")
	{
		MyCaptureKinect1::frame2PointCloud(color_frame, depth_frame, pc, camera_intrinsics_k[camera_id], fsetting);
	}

	pcl::transformPointCloud(pc, pc, metadata.pc_transforms[camera_id]);
	pcl::transformPointCloud(pc, pc, metadata.ref_cam_transform);

}

void MyFileIO::startPcReader()
{
	closeFiles();

	int i;
	std::string filepath;

	// make frame index from timestamp data  -- !!code repetition!!

	for (auto fi : frame_index) fi.clear();
	frame_index.clear();

	frame_index.resize(metadata.num_camera);
	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".pc.ts." + std::to_string(i + 1) + ".txt";
		std::ifstream ifs(filepath, std::ios::in);

		FrameIndex fi;

		while (!ifs.eof())
		{
			ifs >> fi.ts >> fi.fpos;

			frame_index[i].push_back(fi);
		}

	}

	// open input frame data
	for (i = 0; i < metadata.num_camera; i++)
	{
		filepath = data_dir + session_name + ".pc.frame." + std::to_string(i + 1) + ".bin";
		std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
		ifs_framedata.push_back(std::move(ifs));
	}
}

void MyFileIO::readPcFrame(pcl::PointCloud<pcl::PointXYZRGB>& pc, int camera_id, size_t frame_id)
{
	ifs_framedata[camera_id].seekg(frame_index[camera_id][frame_id].fpos, std::ios_base::beg);

	decodePc(pc, ifs_framedata[camera_id]);
	
}

void MyFileIO::startMergedPcReader()
{
	closeFiles();

	int i;
	std::string filepath;

	// make frame index from timestamp data  -- !!code repetition!!
	for (auto fi : frame_index) fi.clear();
	frame_index.clear();

	frame_index.resize(1);
	{
		filepath = data_dir + session_name + ".mrgpc.ts.txt";
		std::ifstream ifs(filepath, std::ios::in);

		FrameIndex fi;

		while (!ifs.eof())
		{
			ifs >> fi.ts >> fi.fpos;

			frame_index[0].push_back(fi);
		}
	}

	// open input frame data -- !!code repetition!!

	{
		filepath = data_dir + session_name + ".mrgpc.frame.bin";
		std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
		ifs_framedata.push_back(std::move(ifs));
	}
}

void MyFileIO::readMergedPcFrame(pcl::PointCloud<pcl::PointXYZRGB>& pc, pcl::PointCloud<pcl::Normal>& pc_n, size_t frame_id)
{
	ifs_framedata[0].seekg(frame_index[0][frame_id].fpos, std::ios_base::beg);

	decodePc(pc, pc_n, ifs_framedata[0]);
}

void MyFileIO::start2DVideoReader()
{
	int i;
	std::vector<bool> vid_exist;

	cv_videocapture.clear();
	cv_videocapture.resize(metadata.num_camera);
	check2DVideoExist(vid_exist);

	for (i = 0; i < metadata.num_camera; i++)
	{
		if (!vid_exist[i]) 
		{
			cv_videocapture[i] = NULL;
			continue;
		}

		std::string filepath;
		filepath = data_dir + session_name + ".2dvideo." + std::to_string(i + 1) + ".avi";

		cv_videocapture[i] = std::shared_ptr<cv::VideoCapture>(new cv::VideoCapture(filepath));

	}
}

void MyFileIO::read2DVideoFrame(cv::Mat & vid_frame, int camera_id, size_t frame_id)
{
	if (cv_videocapture[camera_id] == NULL)
	{
		vid_frame = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
	}
	else
	{
		size_t crnt_frame = cv_videocapture[camera_id]->get(CV_CAP_PROP_POS_FRAMES);

		if (crnt_frame != frame_id) cv_videocapture[camera_id]->set(CV_CAP_PROP_POS_FRAMES, frame_id);

		*cv_videocapture[camera_id] >> vid_frame;

		if (vid_frame.empty())
		{
			vid_frame = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
		}

	}
}

void MyFileIO::closeFiles()
{
	ofs_framedata.clear(); ofs_framedata.shrink_to_fit();

	ofs_timestamp.clear(); ofs_timestamp.shrink_to_fit();

	ifs_framedata.clear(); ifs_framedata.shrink_to_fit();

	cv_videowriters.clear(); cv_videowriters.shrink_to_fit();
	cv_videocapture.clear(); cv_videocapture.shrink_to_fit();

	camera_intrinsics.clear(); camera_intrinsics.shrink_to_fit();
	camera_intrinsics_k.clear(); camera_intrinsics_k.shrink_to_fit();
}

void MyFileIO::export2DVideoFromRGBD(int fourcc)
{
	if (!checkRgbdDataExist())
	{
		std::cout << "error in exporting 2D video: no RGBD data was found." << std::endl;
		return;
	}

	startRgbdReader();

	int camera_id;
	long frame_id;

	std::cout << "exporting 2D video" << std::endl;
	for (frame_id = 0; frame_id < getNumFrame(); frame_id++)
	{
		for (camera_id = 0; camera_id < metadata.num_camera; camera_id++)
		{
			cv::Mat color_frame, depth_frame;
			readRgbdFrame(color_frame, depth_frame, camera_id, frame_id);

			if (frame_id == 0) start2DVideoWriter(camera_id, color_frame.size().width, color_frame.size().height, fourcc);

			write2DVideoFrame(color_frame, camera_id);
		}
		if (getNumFrame() > 50 && frame_id % (getNumFrame() / 50) == 0) std::cout << "#";
	}
	
	std::cout << " - finished." << std::endl;

}

void MyFileIO::saveCameraIntrinsics(RsCameraIntrinsics & ci, FILE *fo)
{
	saveRsIntrinsic(ci.depth_intrin, fo);
	saveRsExtrinsic(ci.depth_to_color, fo);
	saveRsIntrinsic(ci.color_intrin, fo);
	fwrite(&ci.scale, sizeof(float), 1, fo);
}

void MyFileIO::loadCameraIntrinsics(RsCameraIntrinsics & ci, FILE *fi)
{
	loadRsIntrinsic(ci.depth_intrin, fi);
	loadRsExtrinsic(ci.depth_to_color, fi);
	loadRsIntrinsic(ci.color_intrin, fi);
	fread(&ci.scale, sizeof(float), 1, fi);
}

void MyFileIO::saveCameraIntrinsics(INuiCoordinateMapper* cm, FILE *fo)
{
	ULONG cm_size;
	void *cm_data;
	cm->GetColorToDepthRelationalParameters(&cm_size, &cm_data);

	fwrite(&cm_size, sizeof(ULONG), 1, fo);
	fwrite(cm_data, cm_size, 1, fo);
}

void MyFileIO::loadCameraIntrinsics(INuiCoordinateMapper** cm, FILE *fi)
{
	ULONG cm_size;

	fread(&cm_size, sizeof(ULONG), 1, fi);
	BYTE *cm_buf = new BYTE[cm_size];
	fread((void*)cm_buf, cm_size, 1, fi);

	NuiCreateCoordinateMapperFromParameters(cm_size, (void*)cm_buf, cm);

	delete[] cm_buf;
}

void MyFileIO::saveRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, std::ostream & os)
{
	std::vector<unsigned char> buf;
	std::vector<int> cmp;
	size_t s;
	
	// write RGB image as jpg (lossy)
	cmp.push_back(cv::IMWRITE_JPEG_QUALITY);
	cmp.push_back(95);

	cv::imencode(".jpg", color_frame, buf, cmp);

	s = buf.size();
	os.write((char*)&s, sizeof(size_t));
	os.write((char*)buf.data(), buf.size());

	// write depth image as png (lossless)
	buf.clear();
	cmp.clear();

	cmp.push_back(cv::IMWRITE_PNG_COMPRESSION);
	cmp.push_back(1);

	cv::imencode(".png", depth_frame, buf, cmp);

	s = buf.size();
	os.write((char*)&s, sizeof(size_t));
	os.write((char*)buf.data(), buf.size());
}

void MyFileIO::loadRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, std::istream & is)
{
	std::vector<unsigned char> buf;
	size_t s;

	// read RGB image 

	is.read((char*)&s, sizeof(size_t));
	buf.resize(s);
	is.read((char*)buf.data(), buf.size());

	color_frame = cv::imdecode(cv::Mat(buf), CV_LOAD_IMAGE_COLOR);

	// read depth image 

	is.read((char*)&s, sizeof(size_t));
	buf.resize(s);
	is.read((char*)buf.data(), buf.size());

	depth_frame = cv::imdecode(cv::Mat(buf), CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
}

void MyFileIO::getR200Intrinsics(std::vector<RsCameraIntrinsics>& intrinsics)
{
	intrinsics.clear();

	for (auto ci : camera_intrinsics)
	{
		intrinsics.push_back(ci);
	}
}

void MyFileIO::getSessionName(std::string & name)
{
	name = session_name;
}

void MyFileIO::getDataDir(std::string & ddir)
{
	ddir = data_dir;
}

void MyFileIO::encodePc(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::ostream & os)
{
	size_t n_pt = pc.size();

	os.write((char*)&n_pt, sizeof(size_t));
	for (auto p : pc)
	{
		int16_t x = floor(p.x * 2000);
		int16_t y = floor(p.y * 2000);
		int16_t z = floor(p.z * 2000);
		os.write((char*)&x, sizeof(int16_t));
		os.write((char*)&y, sizeof(int16_t));
		os.write((char*)&z, sizeof(int16_t));
		os.write((char*)&p.r, sizeof(uint8_t));
		os.write((char*)&p.g, sizeof(uint8_t));
		os.write((char*)&p.b, sizeof(uint8_t));
	}
}

void MyFileIO::decodePc(pcl::PointCloud<pcl::PointXYZRGB>& pc, std::istream & is)
{
	size_t n_pt;
	is.read((char*)&n_pt, sizeof(size_t));

	pc.clear();
	for (int i = 0; i < n_pt; i++)
	{
		pcl::PointXYZRGB p;

		int16_t x, y, z;
		is.read((char*)&x, sizeof(int16_t));
		is.read((char*)&y, sizeof(int16_t));
		is.read((char*)&z, sizeof(int16_t));
		p.x = ((float)x) / 2000.0;
		p.y = ((float)y) / 2000.0;
		p.z = ((float)z) / 2000.0;

		is.read((char*)&p.r, sizeof(uint8_t));
		is.read((char*)&p.g, sizeof(uint8_t));
		is.read((char*)&p.b, sizeof(uint8_t));

		pc.push_back(p);
	}
}

void MyFileIO::encodePc(pcl::PointCloud<pcl::PointXYZRGB>& pc, pcl::PointCloud<pcl::Normal>& pc_n, std::ostream & os)
{
	size_t n_pt = pc.size();

	os.write((char*)&n_pt, sizeof(size_t));
	for (int i=0; i< n_pt; i++)
	{
		pcl::PointXYZRGB p = pc[i];
		pcl::Normal p_n = pc_n[i];

		int16_t x = floor(p.x * 2000);
		int16_t y = floor(p.y * 2000);
		int16_t z = floor(p.z * 2000);
		os.write((char*)&x, sizeof(int16_t));
		os.write((char*)&y, sizeof(int16_t));
		os.write((char*)&z, sizeof(int16_t));

		os.write((char*)&p.r, sizeof(uint8_t));
		os.write((char*)&p.g, sizeof(uint8_t));
		os.write((char*)&p.b, sizeof(uint8_t));

		int8_t nx = floor(p_n.normal_x * 100);
		int8_t ny = floor(p_n.normal_y * 100);
		int8_t nz = floor(p_n.normal_z * 100);
		os.write((char*)&nx, sizeof(int8_t));
		os.write((char*)&ny, sizeof(int8_t));
		os.write((char*)&nz, sizeof(int8_t));
	}
}

void MyFileIO::decodePc(pcl::PointCloud<pcl::PointXYZRGB>& pc, pcl::PointCloud<pcl::Normal>& pc_n, std::istream & is)
{
	size_t n_pt;
	is.read((char*)&n_pt, sizeof(size_t));

	pc.clear();
	pc_n.clear();
	for (int i = 0; i < n_pt; i++)
	{
		pcl::PointXYZRGB p;
		pcl::Normal p_n;

		int16_t x, y, z;
		is.read((char*)&x, sizeof(int16_t));
		is.read((char*)&y, sizeof(int16_t));
		is.read((char*)&z, sizeof(int16_t));
		p.x = ((float)x) / 2000.0;
		p.y = ((float)y) / 2000.0;
		p.z = ((float)z) / 2000.0;

		is.read((char*)&p.r, sizeof(uint8_t));
		is.read((char*)&p.g, sizeof(uint8_t));
		is.read((char*)&p.b, sizeof(uint8_t));

		int8_t nx, ny, nz;
		is.read((char*)&nx, sizeof(int8_t));
		is.read((char*)&ny, sizeof(int8_t));
		is.read((char*)&nz, sizeof(int8_t));

		// normalize again to correct compression error
		Eigen::Vector3f v(((float)nx) / 100.0, ((float)ny) / 100.0, ((float)nz) / 100.0);
		v.normalize();

		p_n.normal_x = v.x();
		p_n.normal_y = v.y();
		p_n.normal_z = v.z();


		pc.push_back(p);
		pc_n.push_back(p_n);
	}
}

void MyFileIO::saveRsIntrinsic(rs::intrinsics & intrin, FILE *fo)
{
	rs_intrinsics & intrin2 = intrin;

	fwrite(&intrin2.width, sizeof(int), 1, fo);
	fwrite(&intrin2.height, sizeof(int), 1, fo);
	fwrite(&intrin2.ppx, sizeof(float), 1, fo);
	fwrite(&intrin2.ppy, sizeof(float), 1, fo);
	fwrite(&intrin2.fx, sizeof(float), 1, fo);
	fwrite(&intrin2.fy, sizeof(float), 1, fo);
	fwrite(&intrin2.model, sizeof(rs_distortion), 1, fo);
	fwrite(&intrin2.coeffs, sizeof(float), 5, fo);
}

void MyFileIO::loadRsIntrinsic(rs::intrinsics & intrin, FILE *fi)
{
	rs_intrinsics & intrin2 = intrin;

	fread(&intrin2.width, sizeof(int), 1, fi);
	fread(&intrin2.height, sizeof(int), 1, fi);
	fread(&intrin2.ppx, sizeof(float), 1, fi);
	fread(&intrin2.ppy, sizeof(float), 1, fi);
	fread(&intrin2.fx, sizeof(float), 1, fi);
	fread(&intrin2.fy, sizeof(float), 1, fi);
	fread(&intrin2.model, sizeof(rs_distortion), 1, fi);
	fread(&intrin2.coeffs, sizeof(float), 5, fi);

}

void MyFileIO::saveRsExtrinsic(rs::extrinsics & extrin, FILE *fo)
{
	rs_extrinsics & extrin2 = extrin;

	fwrite(&extrin2.rotation, sizeof(float), 9, fo);
	fwrite(&extrin2.translation, sizeof(float), 3, fo);

}

void MyFileIO::loadRsExtrinsic(rs::extrinsics & extrin, FILE *fi)
{
	rs_extrinsics & extrin2 = extrin;

	fread(&extrin2.rotation, sizeof(float), 9, fi);
	fread(&extrin2.translation, sizeof(float), 3, fi);

}
