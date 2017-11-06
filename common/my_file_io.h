#pragma once

#include <string>
#include "my_capture_kinect1.h"
#include "my_capture_r200.h"
#include "my_metadata.h"
#include "rodent_tracker.h"

#include <pcl/compression/octree_pointcloud_compression.h>

class MyFileIO
{
public:

	enum DataType
	{
		DATA_TYPE_NONE = -1,
		DATA_TYPE_RGBD = 0,
		DATA_TYPE_PC = 1,
		DATA_TYPE_MERGED_PC = 2,
	};

	struct FrameIndex
	{
		double ts;
		fpos_t fpos;
	};

	MyMetadata metadata;

	MyFileIO();
	MyFileIO(const std::string & metadatafilepath);														// for opening existing data
	MyFileIO(const std::string & path_data_dir, const std::string & name_session, MyMetadata & md);		// for recording new data
	~MyFileIO();

	void saveMetadata();
	void loadMetadata();

	void saveTrackingResult(RodentTrackerResult & result);
	void loadTrackingResult(RodentTrackerResult & result);

	void saveTrackingParam(RodentTrackerParam & param);
	void loadTrackingParam(RodentTrackerParam & param);

	bool checkRgbdDataExist();
	bool checkPcDataExist();
	bool checkMergedPcDataExist();
	bool checkTrackingResultExist();
	bool checkTrackingParamExist();
	void check2DVideoExist(std::vector<bool> & vid_exist);

	void saveCameraIntrinsics(std::shared_ptr<MyCapture> & cap);
	void saveCameraIntrinsics(std::shared_ptr<MyCaptureR200> & cap);
	void saveCameraIntrinsics(std::shared_ptr<MyCaptureKinect1> & cap);

	void startRgbdWriter();
	void writeRgbdFrame(cv::Mat color_frame, cv::Mat depth_frame, double timestamp, int camera_id);

	void startPcWriter();
	void writePcFrame(pcl::PointCloud<pcl::PointXYZRGB> &pc, double timestamp, int camera_id);

	void start2DVideoWriter(int camera_id, int w = 320, int h = 240, int fourcc = CV_FOURCC('X', 'V', 'I', 'D'));
	void write2DVideoFrame(cv::Mat color_frame, int camera_id);

	void preprosessData(float gridsize, std::vector<bool> cam_enable = std::vector<bool>());

	void startRgbdReader();
	long getNumFrame() { if (frame_index.size() == 0) return 0; else return frame_index[0].size(); };
	void readRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, int camera_id, size_t frame_id);
	void rgbdFrame2PointCloud(const cv::Mat & color_frame, const cv::Mat & depth_frame,
							  pcl::PointCloud<pcl::PointXYZRGB> & pc, const int camera_id);

	double getTimestamp(size_t frame_id, size_t camera_id = 0) { return frame_index[camera_id][frame_id].ts;  };

	void startPcReader();
	void readPcFrame(pcl::PointCloud<pcl::PointXYZRGB> &pc, int camera_id, size_t frame_id);

	void startMergedPcReader();
	void readMergedPcFrame(pcl::PointCloud<pcl::PointXYZRGB> &pc, pcl::PointCloud<pcl::Normal> &pc_n, size_t frame_id);

	void start2DVideoReader();
	void read2DVideoFrame(cv::Mat & vid_frame, int camera_id, size_t frame_id);

	std::string getDataPathHeader() { return data_dir + session_name;  };

	void closeFiles();

	void export2DVideoFromRGBD(int fourcc = CV_FOURCC('X', 'V', 'I', 'D'));
	
	static void saveCameraIntrinsics(RsCameraIntrinsics & ci, FILE *fo);
	static void loadCameraIntrinsics(RsCameraIntrinsics & ci, FILE *fi);

	static void saveCameraIntrinsics(INuiCoordinateMapper* cm, FILE *fo);
	static void loadCameraIntrinsics(INuiCoordinateMapper** cm, FILE *fi);

	static void saveRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, std::ostream & os);

	static void loadRgbdFrame(cv::Mat & color_frame, cv::Mat & depth_frame, std::istream & is);

	void getR200Intrinsics(std::vector<RsCameraIntrinsics> &intrinsics);

	void getSessionName(std::string & name);
	void getDataDir(std::string & ddir);

	void encodePc(pcl::PointCloud<pcl::PointXYZRGB> &pc, std::ostream & os);
	void decodePc(pcl::PointCloud<pcl::PointXYZRGB> &pc, std::istream & is);

	void encodePc(pcl::PointCloud<pcl::PointXYZRGB> &pc, pcl::PointCloud<pcl::Normal>& pc_n, std::ostream & os);
	void decodePc(pcl::PointCloud<pcl::PointXYZRGB> &pc, pcl::PointCloud<pcl::Normal>& pc_n, std::istream & is);


private:

	std::string data_dir;
	std::string session_name;

	std::vector<std::ofstream> ofs_framedata;
	std::vector<std::ofstream> ofs_timestamp;

	std::vector<std::vector<FrameIndex>> frame_index;

	std::vector<std::ifstream> ifs_framedata;

	std::vector<std::shared_ptr<cv::VideoWriter>> cv_videowriters;
	cv::Size cv_videowriters_size;

	std::vector<std::shared_ptr<cv::VideoCapture>> cv_videocapture;

	std::vector<RsCameraIntrinsics> camera_intrinsics;
	std::vector<INuiCoordinateMapper*> camera_intrinsics_k;

	static void saveRsIntrinsic(rs::intrinsics & intrin, FILE *fo);
	static void loadRsIntrinsic(rs::intrinsics & intrin, FILE *fi);
	static void saveRsExtrinsic(rs::extrinsics & extrin, FILE *fo);
	static void loadRsExtrinsic(rs::extrinsics & extrin, FILE *fi);

};

