#include "../common/my_file_io.h"

#include <vector>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <librealsense2\rs.hpp>

// loadbagfile: load a bag file to a pipeline
void loadbagfile(rs2::pipeline & pipe, std::string filename)
{
	rs2::config cfg;
	cfg.enable_device_from_file(filename);
	pipe.start(cfg);

	auto device = pipe.get_active_profile().get_device();
	rs2::playback playback = device.as<rs2::playback>();
	playback.pause();
	playback.set_real_time(false);
}

// readframe: read the next frame of a bag-file-playback-pipeline
int readframe(rs2::pipeline & pipe, size_t & framenumber, rs2::frameset & frameset)
{
	auto device = pipe.get_active_profile().get_device();
	rs2::playback playback = device.as<rs2::playback>();

	playback.resume();
	frameset = pipe.wait_for_frames();
	playback.pause();

	if (frameset[0].get_frame_number() < framenumber) return EOF;

	framenumber = frameset[0].get_frame_number();

	return 0;
}

// readframeto: read the frame just after the specified timestamp.  
int readframeto(double timestamp, rs2::pipeline & pipe, size_t & framenumber, rs2::frameset & frameset)
{
	while (true)
	{
		if (timestamp <= frameset.get_timestamp()) return 0;

		int r = readframe(pipe, framenumber, frameset);
		if (r == EOF) return EOF;
	}

	return 0;
}

int main()
{
	MyFileIO fio("");

	std::vector<rs2::pipeline> pipes;
	std::vector<size_t> framenumbers;
	std::vector<rs2::frameset> framesets;
	std::vector<RsCameraIntrinsics2> cis;


	// step 1: load bag files //////////////////////////////////////////////////////

	int num_file = fio.metadata.num_camera;
	std::string data_dir, session_name;
	fio.getDataDir(data_dir);
	fio.getSessionName(session_name);

	pipes.clear();
	for (int i = 0; i < num_file; i++)
	{
		rs2::pipeline pipe;
		std::string filepath = data_dir + session_name + ".rosbag." + std::to_string(i + 1) + ".bag";
		loadbagfile(pipe, filepath);
		pipes.push_back(pipe);
		framenumbers.push_back(0ULL);

		RsCameraIntrinsics2 ci;
		auto prof = pipe.get_active_profile();
		ci.color_intrin = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
		ci.depth_intrin = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
		ci.depth_to_color = prof.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(prof.get_stream(RS2_STREAM_COLOR));
		ci.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();

		cis.push_back(ci);
	}

	for (int i = 0; i < num_file; i++)
	{
		std::string filepath = data_dir + session_name + ".camintrin." + std::to_string(i + 1) + ".bin";
		FILE *fp = fopen(filepath.c_str(), "wb");

		MyFileIO::saveCameraIntrinsics(cis[i], fp);

		fclose(fp);
	}


	// step 1 end /////////////////////////////////////////////////////////////////

	// step 2: skip frames until the time when every camera is ready (t_start). ///
	for (int i = 0; i < num_file; i++)
	{
		rs2::frameset frameset;
		readframe(pipes[i], framenumbers[i], frameset);
		framesets.push_back(frameset);
	}

	double t_start = 0;
	for (int i = 0; i < num_file; i++)
	{
		if (t_start < framesets[i].get_timestamp()) t_start = framesets[i].get_timestamp();
	}

	for (int i = 0; i < num_file; i++)
	{
		readframeto(t_start, pipes[i], framenumbers[i], framesets[i]);
	}
	// step 2 end /////////////////////////////////////////////////////////////////

	// step 3: read and convert the synchronized frames until the file end ////////
	fio.startRgbdWriter();


	int framecnt = 0;
	while (1)
	{
		int r;
		bool flag_end = false;

		r = readframe(pipes[0], framenumbers[0], framesets[0]);
		if (r == EOF) flag_end = true;

		for (int i = 1; i < num_file; i++)
		{
			r = readframeto(framesets[0].get_timestamp(), pipes[i], framenumbers[i], framesets[i]);
			if (r == EOF) flag_end = true;
		}
		if (flag_end) break;

		for (int i = 1; i < num_file; i++)
		{
			// show time lag between synchronized frames of cameras (which should be < 1/fps).
			std::cout << "Frame count: " << framecnt << ", Camera No: " << i << ", time lag compared with Camera0 = " << (size_t)(framesets[i].get_timestamp() - framesets[0].get_timestamp()) << "ms" << std::endl;
		}

		// save as rgbd data
		boost::thread_group thr_grp;
		
		for (int i = 0; i < num_file; i++)
		{
			rs2::frame color_frame, depth_frame;
			color_frame = framesets[i].get_color_frame();
			depth_frame = framesets[i].get_depth_frame();
		
			cv::Mat color, depth;
			color = cv::Mat(cv::Size(cis[i].color_intrin.width, cis[i].color_intrin.height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
			depth = cv::Mat(cv::Size(cis[i].depth_intrin.width, cis[i].depth_intrin.height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
			
			thr_grp.create_thread(boost::bind(&MyFileIO::writeRgbdFrame, &fio, color, depth, framesets[0].get_timestamp(), i));

		}

		thr_grp.join_all();

		framecnt++;
	}

	fio.closeFiles();

	return 0;
}