
#include <windows.h>
#include <direct.h>
#include <shlwapi.h>

#include <opencv2/opencv.hpp>

#include "gl/freeglut.h"

#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "btBulletDynamicsCommon.h"
#include <pcl/octree/octree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <time.h>
#include <stdio.h>
#include <ShlObj.h>

#include "../common/my_physics_sim.h"
#include "../common/my_metadata.h"
#include "../common/app_common.h"
#include "../common/rodent_tracker.h"
#include "../common/my_file_io.h"
#include "../common/my_gl_texture.h"


pcl::PointCloud<pcl::PointXYZRGBNormal> pc_crntframe;

int i_preframe;
int i_crntframe;				// currently processed (displayed) frame

int tgtpart;					// target body part for manual modification of position of body part

bool vplay;

pcl::PointCloud<pcl::PointXYZ> pos_cursor;

std::shared_ptr<MyFileIO> fileIO;

std::shared_ptr<RodentTracker> tracker;
std::shared_ptr<RodentTrackerParam> tracker_param;
RodentTrackerResult tracker_result;

std::vector<btVector3> nose_pos;
std::vector<bool> nose_pos_enable;

bool disp_every_5th_frame = false;
float play_speed = 1.0;

bool preproc_finished = false;
bool view_show_model = true;
bool view_show_nose = false;
bool view_show_axis = true;
float view_pointsize = 5.0;
float view_transparency = 0.0;
int view_model_coloring = 0;	// 0=normal, 1=whiter head, 2=darker body

std::vector<bool> colorvideo_exist;
MyGlTexture colorvideo_monitor_tex;
int colorvideo_monitor_cam_id = 0;

bool online_mode = false;
bool recording = false;
size_t rec_frame_cnt;
double rec_ts0;
std::string camsetting_filepath;
double ts_online;
FILE *fp_online_result;


void getColor(int animal_id, float *clr)
{
	int ii = (animal_id + 1) % 8;
	float g = ((ii / 4) % 2) * 1.0,  b = ((ii / 2) % 2) * 1.0, r = (ii % 2) * 1.0;

	clr[0] = r; clr[1] = g; clr[2] = b; clr[3] = 1.0;
}

void getColorBody(int animal_id, float *clr)
{
	getColor(animal_id, clr);

	if (view_model_coloring == 2)
	{
		clr[0] = clr[0] * 3 / 8;
		clr[1] = clr[1] * 3 / 8;
		clr[2] = clr[2] * 3 / 8;
	}
}

void getColorHead(int animal_id, float *clr)
{
	getColor(animal_id, clr);

	if (view_model_coloring == 1)
	{
		clr[0] = clr[0] + 0.5;
		clr[1] = clr[1] + 0.5;
		clr[2] = clr[2] + 0.5;
	}
}

bool compFirst(const std::pair<float, int> &a, const std::pair<float, int> &b) {
	return a.first > b.first;
}

void sortPcByDepth(pcl::PointCloud<pcl::PointXYZRGBNormal> &pc_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &pc_out)
{
	int n = pc_in.size();

	std::vector<std::pair<float, int>> depth_list(n);

	GLdouble mv_mat[16];
	GLdouble prj_mat[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, mv_mat);
	glGetDoublev(GL_PROJECTION_MATRIX, prj_mat);
	glGetIntegerv(GL_VIEWPORT, viewport);


	for (int i = 0; i < n; i++)
	{
		double winX, winY, winZ;
		gluProject(pc_in[i].x, pc_in[i].y, pc_in[i].z, mv_mat, prj_mat, viewport, &winX, &winY, &winZ);
		depth_list[i] = std::make_pair(winZ, i);
	}

	std::sort(depth_list.begin(), depth_list.end(), compFirst);

	pc_out.clear();
	for (int i = 0; i < n; i++)
	{
		pc_out.push_back(pc_in[depth_list[i].second]);
	}

}

void calcNosePos(long frame_id)
{
	int i, j;

	for (auto & npe : nose_pos_enable) npe = false;

	if (frame_id > tracker_result.last_frame) return;

	for (int animal_id = 0; animal_id < tracker_result.n_animal; animal_id++)
	{
		btVector3 hd, nk;
		hd = tracker_result.pos[animal_id][frame_id].hd.getOrigin();
		nk = tracker_result.pos[animal_id][frame_id].n.getOrigin();

		int max_i[5] = { 0, 0, 0, 0, 0 };
		float max_d[5] = { -1.0, -1.0, -1.0, -1.0, -1.0 };

		int min_max_i = 0;
		float min_max_d = -1.0;

		float d_thresh = hd.distance(nk) * 1.2;

		for (i = 0; i<pc_crntframe.size(); i++)
		{
			btVector3 p(pc_crntframe[i].x, pc_crntframe[i].y, pc_crntframe[i].z);

			if (hd.distance(p) > d_thresh) continue;

			btVector3 hdp = p - hd;
			btVector3 nkhd = hd - nk;

			float d = nkhd.dot(hdp);

			if (d > min_max_d)
			{
				max_d[min_max_i] = d;
				max_i[min_max_i] = i;

				min_max_i = 0;
				min_max_d = max_d[0];
				for (j = 1; j<5; j++)
				{
					if (min_max_d > max_d[j])
					{
						min_max_i = j;
						min_max_d = max_d[j];
					}
				}
			}
		}

		int cnt = 0;
		btVector3 cog(0.0, 0.0, 0.0);
		for (i = 0; i<5; i++)
		{
			if (max_d[i] < 0.0) continue;
			btVector3 p(pc_crntframe[max_i[i]].x, pc_crntframe[max_i[i]].y, pc_crntframe[max_i[i]].z);
			cog += p;
			cnt++;
		}

		if (cnt > 0)
		{
			cog /= (double)cnt;
			nose_pos[animal_id] = cog;
			nose_pos_enable[animal_id] = true;
		}

	}

}

void hsvFilterPc(pcl::PointCloud < pcl::PointXYZRGBNormal> & pc_in,
	pcl::PointCloud < pcl::PointXYZRGBNormal> & pc_out,
	cv::Scalar hsv_min, cv::Scalar hsv_max, bool remove)
{

	int i;
	cv::Mat point_rgb(pc_in.size(), 1, CV_8UC3);
	cv::Mat point_hsv, point_bg;

	for (i = 0; i < pc_in.size(); i++)
	{
		point_rgb.at<cv::Vec3b>(i).val[0] = pc_in[i].b;
		point_rgb.at<cv::Vec3b>(i).val[1] = pc_in[i].g;
		point_rgb.at<cv::Vec3b>(i).val[2] = pc_in[i].r;
	}
	cv::cvtColor(point_rgb, point_hsv, cv::COLOR_BGR2HSV);
	cv::inRange(point_hsv, hsv_min, hsv_max, point_bg);

	pcl::PointCloud<pcl::PointXYZRGBNormal> tmp_pc;
	if (remove)
	{
		for (i = 0; i < pc_in.size(); i++) if (point_bg.at<unsigned char>(i) == 0) tmp_pc.push_back(pc_in[i]);
	}
	else
	{
		for (i = 0; i < pc_in.size(); i++) if (point_bg.at<unsigned char>(i) != 0) tmp_pc.push_back(pc_in[i]);
	}

	pc_out = tmp_pc;
}

void outlierRemoveFilterPc(pcl::PointCloud < pcl::PointXYZRGBNormal> & pc_in,
	pcl::PointCloud < pcl::PointXYZRGBNormal> & pc_out,
	int meanK, float thresh)
{
	if (pc_in.size() == 0) return;

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
	outrem.setInputCloud(pc_in.makeShared());
	outrem.setRadiusSearch(thresh);
	outrem.setMinNeighborsInRadius(meanK);
	outrem.filter(pc_out);
}

void readFrame(size_t i_frame, bool update_tracker = true)
{
	pcl::PointCloud<pcl::PointXYZRGB> pc_xyzrgb;
	pcl::PointCloud<pcl::Normal> pc_normal;
	pcl::PointCloud<pcl::PointXYZRGBNormal> pc_tmp;

	if (online_mode) ts_online = fileIO->updateOnlineFrame();
	fileIO->readMergedPcFrame(pc_xyzrgb, pc_normal, i_frame);

	pcl::concatenateFields(pc_xyzrgb, pc_normal, pc_tmp);

	pcl::CropBox<pcl::PointXYZRGBNormal> cb;
	cb.setMin(Eigen::Vector4f(fileIO->metadata.roi.x[0], fileIO->metadata.roi.y[0], fileIO->metadata.roi.z[0], 1.0));
	cb.setMax(Eigen::Vector4f(fileIO->metadata.roi.x[1], fileIO->metadata.roi.y[1], fileIO->metadata.roi.z[1], 1.0));
	cb.setInputCloud(pc_tmp.makeShared());
	cb.filter(pc_tmp);

	if (tracker_param->cf_enable) hsvFilterPc(pc_tmp, pc_tmp, cv::Scalar(tracker_param->cf_hsv_min[0], tracker_param->cf_hsv_min[1], tracker_param->cf_hsv_min[2]), cv::Scalar(tracker_param->cf_hsv_max[0], tracker_param->cf_hsv_max[1], tracker_param->cf_hsv_max[2]), tracker_param->cf_inc==0);
	if (tracker_param->orf_enable) outlierRemoveFilterPc(pc_tmp, pc_tmp, tracker_param->orf_meanK, tracker_param->orf_thresh);

	pc_crntframe = pc_tmp;
	if (update_tracker) tracker->setPointCloud(pc_crntframe);
}

void exportNosePos()
{
	OPENFILENAMEA ofn;
	char szFile[MAX_PATH];
	std::string session_name;
	fileIO->getSessionName(session_name);
	sprintf(szFile, "%s.nosepos.csv", session_name.c_str());
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(OPENFILENAMEA);
	ofn.lpstrFilter = "Nose positions (*.csv)\0*.csv\0";
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_OVERWRITEPROMPT;
	ofn.lpstrDefExt = "csv";
	ofn.lpstrTitle = "Save";

	GetSaveFileNameA(&ofn);

	FILE *fp = fopen(ofn.lpstrFile, "w");

	std::cout << "calculating and exporting nose positions..." << std::endl;
	for (i_crntframe = 0; i_crntframe < fileIO->getNumFrame(); i_crntframe++)
	{
		readFrame(i_crntframe);

		fprintf(fp, "%lf, ", fileIO->getTimestamp(i_crntframe));
		
		calcNosePos(i_crntframe);

		for (int i = 0; i < tracker->sm.size(); i++)
		{
			if (nose_pos_enable[i]) fprintf(fp, "%f, %f, %f, ", nose_pos[i].x(), nose_pos[i].y(), nose_pos[i].z());
			else fprintf(fp, ", , , ");
		}

		fprintf(fp, "\n");

		if ((i_crntframe + 1) % (size_t)(fileIO->getNumFrame() / 50) == 0) std::cout << "#";

	}
	std::cout << " - finished" << std::endl;

	fclose(fp);
}

void updateColorVideoFrame()
{
	cv::Mat colorvideo_frame;

	fileIO->read2DVideoFrame(colorvideo_frame, colorvideo_monitor_cam_id, i_crntframe);
	colorvideo_monitor_tex.update(colorvideo_frame);
}

bool checkModelInRoi(int id)
{
	if (id >= tracker_param->n_animal) return false;

	float roi_center_x = (fileIO->metadata.roi.x[0] + fileIO->metadata.roi.x[1]) / 2.0;
	float roi_center_z = (fileIO->metadata.roi.z[0] + fileIO->metadata.roi.z[1]) / 2.0;
	float roi_length_x = fileIO->metadata.roi.x[1] - fileIO->metadata.roi.x[0];
	float roi_length_z = fileIO->metadata.roi.z[1] - fileIO->metadata.roi.z[0];

	btVector3 pos = tracker->sm[id].body_t->getCenterOfMassPosition();

	if (pos.x() < fileIO->metadata.roi.x[0] - roi_length_x / 4.0 ||
		pos.x() > fileIO->metadata.roi.x[1] + roi_length_x / 4.0 ||
		pos.z() < fileIO->metadata.roi.z[0] - roi_length_z / 4.0 ||
		pos.z() > fileIO->metadata.roi.z[1] + roi_length_z / 4.0)
	{
		return false;
	}
		
	return true;


}

void initModelPos(int id, float shift)
{
	if (id >= tracker_param->n_animal) return;

	float roi_center_x = (fileIO->metadata.roi.x[0] + fileIO->metadata.roi.x[1]) / 2.0;
	float roi_center_z = (fileIO->metadata.roi.z[0] + fileIO->metadata.roi.z[1]) / 2.0;
	float roi_length_x = fileIO->metadata.roi.x[1] - fileIO->metadata.roi.x[0];
	float roi_length_z = fileIO->metadata.roi.z[1] - fileIO->metadata.roi.z[0];

	tracker->setModelPos(id, roi_center_x + roi_length_x * shift, tracker_param->h_floor, roi_center_z + roi_length_z*(float)id, 0, 0);
}

void initModelPos(void)
{
	//initialize all positions of the models 
	for (int i = 0; i < tracker_param->n_animal; i++)
	{
		initModelPos(i, 0.0);
	}
}

void initTrackingResult(void)
{
	if (!online_mode)
	{
		tracker_result.initialize(tracker_param->n_animal, fileIO->getNumFrame());
	}
}

void initTracking()
{
	fileIO->startMergedPcReader();

	tracker_param = std::shared_ptr<RodentTrackerParam>(new RodentTrackerParam());
	tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));

	i_crntframe = 0;

	readFrame(i_crntframe);

	if (!online_mode && fileIO->checkTrackingResultExist())
	{
		fileIO->loadMetadata();
		fileIO->loadTrackingParam(*tracker_param);

		tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
		tracker->setPointCloud(pc_crntframe);
		initTrackingResult();

		fileIO->loadTrackingResult(tracker_result);
	}
	else
	{
		initTrackingResult();
		initModelPos();
	}

}

void showPC(void)
{
	int i;

	pcl::PointCloud<pcl::PointXYZRGBNormal> pc_to_show;

	if (view_transparency > 0.0)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		sortPcByDepth(pc_crntframe, pc_to_show);
	}
	else
	{
		pc_to_show = pc_crntframe;
	}


	glDisable(GL_LIGHTING);
	
	glPointSize(view_pointsize);
	GLubyte alpha = floor((1.0 - view_transparency)*255.0);
	
	glBegin(GL_POINTS);
	for (i = 0; i<pc_to_show.size(); i++)
	{
		glColor4ub(pc_to_show[i].r, pc_to_show[i].g, pc_to_show[i].b, alpha);
		glVertex3f(pc_to_show[i].x, pc_to_show[i].y, pc_to_show[i].z);
	}
	glEnd();

	glEnable(GL_LIGHTING);

	if (view_transparency > 0.0)
	{
		glDisable(GL_BLEND);
	}
}

void showModel(bool sel)
{
	if (!view_show_model) return;

	int i, j;
	float clr_head[4];
	float clr_body[4];

	glInitNames();
	for (i = 0; i < tracker->sm.size(); i++)
	{
		if (!checkModelInRoi(i)) continue;

		getColorHead(i, clr_head);
		getColorBody(i, clr_body);

		glMaterialfv(GL_FRONT, GL_DIFFUSE, clr_head);

		btVector3 b_pos;

		if (sel) glPushName(i * 4);
		glPushMatrix();
		b_pos = tracker->sm[i].body_hd->getCenterOfMassPosition();
		glTranslatef(b_pos.x(), b_pos.y(), b_pos.z());
		glutSolidSphere(tracker->sm[i].params.hd_rbody, 8, 8);
		glPopMatrix();
		if (sel) glPopName();

		glMaterialfv(GL_FRONT, GL_DIFFUSE, clr_body);	

		if (sel) glPushName(i * 4 + 1);
		glPushMatrix();
		b_pos = tracker->sm[i].body_n->getCenterOfMassPosition();
		glTranslatef(b_pos.x(), b_pos.y(), b_pos.z());
		glutSolidSphere(tracker->sm[i].params.n_rbody, 8, 8);
		glPopMatrix();
		if (sel) glPopName();

		if (sel) glPushName(i * 4 + 2);
		glPushMatrix();
		b_pos = tracker->sm[i].body_t->getCenterOfMassPosition();
		glTranslatef(b_pos.x(), b_pos.y(), b_pos.z());
		glutSolidSphere(tracker->sm[i].params.t_rbody, 8, 8);
		glPopMatrix();
		if (sel) glPopName();

		if (sel) glPushName(i * 4 + 3);
		glPushMatrix();
		b_pos = tracker->sm[i].body_hp->getCenterOfMassPosition();
		glTranslatef(b_pos.x(), b_pos.y(), b_pos.z());
		glutSolidSphere(tracker->sm[i].params.hp_rbody, 8, 8);
		glPopMatrix();
		if (sel) glPopName();
	}

	if (!sel)
	{
		glDisable(GL_LIGHTING);

		if (tgtpart >= 0)
		{
			btVector3 b_pos;
			float R;
			int id = tgtpart / 4;

			if (checkModelInRoi(id))
			{
				if (tgtpart % 4 == 0) { b_pos = tracker->sm[id].body_hd->getCenterOfMassPosition(); R = tracker->sm[id].params.hd_rbody; }
				if (tgtpart % 4 == 1) { b_pos = tracker->sm[id].body_n->getCenterOfMassPosition(); R = tracker->sm[id].params.n_rbody; }
				if (tgtpart % 4 == 2) { b_pos = tracker->sm[id].body_t->getCenterOfMassPosition(); R = tracker->sm[id].params.t_rbody; }
				if (tgtpart % 4 == 3) { b_pos = tracker->sm[id].body_hp->getCenterOfMassPosition(); R = tracker->sm[id].params.hp_rbody; }

				glPushMatrix();

				glTranslatef(b_pos.x(), b_pos.y(), b_pos.z());
				glColor3f(1.0, 0.0, 0.0);
				glLineWidth(1.0);
				glutWireSphere(R*1.1, 8, 8);

				glPopMatrix();
			}

		}

		glLineWidth(5.0);
		glBegin(GL_LINES);
		for (i = 0; i<tracker->sm.size(); i++)
		{
			if (!checkModelInRoi(i)) continue;
			getColorBody(i, clr_body);

			glColor4fv(clr_body);

			btVector3 v;
			pcl::PointXYZ p1;
			pcl::PointXYZ p2;
			v.setValue(0.0, tracker->sm[i].params.t_rbody, 0.0);
			v = tracker->sm[i].body_t->getCenterOfMassTransform() * v;
			glVertex3f(v.x(), v.y(), v.z());
			v = tracker->sm[i].body_n->getCenterOfMassPosition();
			glVertex3f(v.x(), v.y(), v.z());

			v.setValue(0.0, tracker->sm[i].params.t_rbody, 0.0);
			v = tracker->sm[i].body_t->getCenterOfMassTransform() * v;
			glVertex3f(v.x(), v.y(), v.z());
			v.setValue(0.0, tracker->sm[i].params.hp_rbody, 0.0);
			v = tracker->sm[i].body_hp->getCenterOfMassTransform() * v;
			glVertex3f(v.x(), v.y(), v.z());
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}

}

void showNose()
{
	if (!view_show_nose) return;

	int i;
	float clr[4];

	calcNosePos(i_crntframe);

	for (i = 0; i < tracker->sm.size(); i++)
	{
		getColor(i, clr);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, clr);

		if (nose_pos_enable[i])
		{
			glPushMatrix();
			glTranslatef(nose_pos[i].x(), nose_pos[i].y(), nose_pos[i].z());
			glutSolidSphere(0.01, 8, 8);
			glPopMatrix();
		}
	}
}

void showSetPosCursor(void)
{
	int i;

	if (pos_cursor.size() == 0) return;

	glDisable(GL_LIGHTING);

	if (pos_cursor.size() == 1)
	{
		glPointSize(10.0);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(pos_cursor[0].x, pos_cursor[0].y, pos_cursor[0].z);
	}
	else if (pos_cursor.size() == 2)
	{
		glLineWidth(10.0);
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex3f(pos_cursor[0].x, pos_cursor[0].y, pos_cursor[0].z);
		glColor3f(0.0, 0.0, 1.0);
		glVertex3f(pos_cursor[1].x, pos_cursor[1].y, pos_cursor[1].z);
	}

	glEnd();
	glEnable(GL_LIGHTING);
}

int main(int argc, char **argv)
{	
	for (int i = 1; i < argc; i++) {

		if (strcmp(argv[i], "-online") == 0)
		{
			online_mode = true;
			camsetting_filepath = argv[i + 1];
			i = i + 1;
		}
		
	}

	startApp(argc, argv, "Tracker");

	// --- online mode test code start
	/*MyFileIO fio(camsetting_filepath, true);

	fio.start2DVideoReader();
	fio.startMergedPcReader();

	size_t t = clock();
	while (1)
	{
		fio.updateOnlineFrame();

		cv::Mat clr_frame;
		fio.read2DVideoFrame(clr_frame, 0, 0);

		cv::imshow("test", clr_frame);

		pcl::PointCloud<pcl::PointXYZRGB> pc_xyzrgb;
		pcl::PointCloud<pcl::Normal> pc_normal;

		fio.readMergedPcFrame(pc_xyzrgb, pc_normal, 0);
		
		std::cout << 1000.0/double(clock()-t) << " fps" << std::endl;
		t = clock();

		int key = cv::waitKey(1);
		if (key == 27) break;		//ESC
	}
	cv::destroyAllWindows();
	*/
	// --- online mode test code end

	return 0;
}

void mouseAndKey()
{
	ImGuiIO& io = ImGui::GetIO();
	for (int i = 0; i < tracker->sm.size(); i++) tracker->togglePointForce(i, true);

	if (ImGui::IsKeyReleased(32))
	{
		vplay = !vplay;
	}

	//Mouse
	if (io.WantCaptureMouse) return;	// skip if other gui components using mouse
	
	if (ImGui::IsMouseClicked(0) && io.KeyShift)
	{
		if (pos_cursor.size() == 2)
		{
			pos_cursor.clear();
		}
		else
		{
			GLdouble mvmat[16];
			GLdouble projmat[16];
			GLint vp[4];

			glGetIntegerv(GL_VIEWPORT, vp);
			glGetDoublev(GL_PROJECTION_MATRIX, projmat);
			glGetDoublev(GL_MODELVIEW_MATRIX, mvmat);

			float z;
			GLdouble ox, oy, oz;

			glReadPixels(io.MousePos.x, vp[3] - io.MousePos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);

			if (z < 1.0)
			{
				gluUnProject(io.MousePos.x, vp[3] - io.MousePos.y, z, mvmat, projmat, vp, &ox, &oy, &oz);

				pcl::PointXYZ p;
				p.x = ox; p.y = oy; p.z = oz;
				pos_cursor.push_back(p);
			}
		}
	}
	else if (ImGui::IsMouseClicked(0)  && !io.KeyShift)
	{
		int i;
		int min_i;

		GLuint sel_buf[100];
		glSelectBuffer(100, sel_buf);

		glRenderMode(GL_SELECT);
		setView(true, io.MousePos.x, io.MousePos.y);
		showModel(true);

		GLint hits = glRenderMode(GL_RENDER);

		if (hits > 0)
		{
			unsigned int min_z = UINT_MAX;
			int min_i = -1;
			for (i = 0; i<hits; i++)
			{
				if (sel_buf[i * 4 + 2] < min_z)
				{
					min_z = sel_buf[i * 4 + 2];
					min_i = sel_buf[i * 4 + 3];
				}
			}
			//printf("%d %u \n", min_i, min_z);
			tgtpart = min_i;
		}
		else
		{
			tgtpart = -1;
		}

	}
	else if (ImGui::IsMouseDragging(0) && !io.KeyShift && tgtpart >= 0)
	{
		btRigidBody *body;

		GLdouble mvmat[16];
		GLdouble projmat[16];
		GLint vp[4];
		int yy;

		glGetIntegerv(GL_VIEWPORT, vp);
		glGetDoublev(GL_PROJECTION_MATRIX, projmat);
		glGetDoublev(GL_MODELVIEW_MATRIX, mvmat);

		yy = vp[3] - io.MousePos.y;

		int id = tgtpart / 4;
		if (tgtpart % 4 == 0) body = tracker->sm[id].body_hd;
		if (tgtpart % 4 == 1) body = tracker->sm[id].body_n;
		if (tgtpart % 4 == 2) body = tracker->sm[id].body_t;
		if (tgtpart % 4 == 3) body = tracker->sm[id].body_hp;

		btVector3 p(body->getCenterOfMassPosition());

		GLdouble wx, wy, wz;
		GLdouble ox, oy, oz;
		float d;

		gluProject(p.x(), p.y(), p.z(), mvmat, projmat, vp, &wx, &wy, &wz);

		gluUnProject(io.MousePos.x, yy, wz, mvmat, projmat, vp, &ox, &oy, &oz);

		body->translate(btVector3(ox, oy, oz) - p);

		if (tracker_result.last_frame >= i_crntframe) tracker_result.last_frame = i_crntframe - 1;

		tracker->togglePointForce(id, false);
	}
	else if (ImGui::IsMouseClicked(2))
	{
		vplay = !vplay;
	}
	
	if (io.MouseWheel != 0.0)
	{
		if (io.KeyCtrl)
		{
			if (io.MouseWheel > 0)
			{
				view_pointsize--;
				if (view_pointsize < 1.0) view_pointsize = 1.0;
			}
			if (io.MouseWheel < 0)
			{
				view_pointsize++;
				if (view_pointsize > 20.0) view_pointsize = 20.0;
			}
		}
		else if (vplay)
		{
			if (io.MouseWheel > 0)
			{
				play_speed *= 0.8;
				if (play_speed < 0.1) play_speed = 0.1;
			}
			if (io.MouseWheel < 0)
			{
				play_speed *= 1.2;	
				if (play_speed > 32.0) play_speed = 32.0;
			}
		}
		else
		{
			if (io.MouseWheel > 0 && i_crntframe > 0)
			{
				i_crntframe--;
			}
			if (io.MouseWheel < 0 && i_crntframe < fileIO->getNumFrame() - 1)
			{
				i_crntframe++;
			}
		}
	}

}

void startOnlineResultOutput()
{
	int i;

	// trace output into a csv file 
	std::string data_dir;
	std::string session_name;
	std::string filepath;
	fileIO->getDataDir(data_dir);
	fileIO->getSessionName(session_name);
	filepath = data_dir + session_name + ".result_online.csv";
	fp_online_result = fopen(filepath.c_str(), "w");

	//csv header -- code repetition!!
	fprintf(fp_online_result, "N of animals:, %d \n", tracker->sm.size());
	fprintf(fp_online_result, "frame, time, ");
	for (i = 0; i < tracker->sm.size(); i++)
	{
		fprintf(fp_online_result, "Model-%d enabled, ", i + 1);
	}
	for (i = 0; i < tracker->sm.size(); i++)
	{
		fprintf(fp_online_result, "Head_x, Head_y, Head_z, Neck_x, Neck_y, Neck_z, Trunk_x, Trunk_y, Trunk_z, Hip_x, Hip_y, Hip_z, ");
	}
	fprintf(fp_online_result, "\n");
}

void startRecording()
{
	int i;

	fileIO->saveMetadata();

	fileIO->saveTrackingParam(*tracker_param);

	fileIO->startMergedPcWriter();
	for (i = 0; i < fileIO->metadata.num_camera; i++)
	{
		if (fileIO->metadata.rec_save_2d_vid[i])
		{
			fileIO->start2DVideoWriter(i, fileIO->metadata.rec_2d_vid_res[0], fileIO->metadata.rec_2d_vid_res[1]);
		}
	}

	startOnlineResultOutput();

	recording = true;
	rec_frame_cnt = 0;
	rec_ts0 = ts_online;
}

void stopRecording()
{
	fileIO->closeFiles();
	fclose(fp_online_result);

	recording = false;
}

void initApp()
{
	int i;

	if (online_mode) fileIO = std::make_shared<MyFileIO>(camsetting_filepath, true);
	else fileIO = std::make_shared<MyFileIO>("");

	if (fileIO->checkMergedPcDataExist())
	{
		preproc_finished = true;
		initTracking();
	}

	i_preframe = -1;
	vplay = false;

	fileIO->check2DVideoExist(colorvideo_exist);
	colorvideo_monitor_tex.init();
	for (i = 0; i < fileIO->metadata.num_camera; i++)
	{
		if (colorvideo_exist[i])
		{
			colorvideo_monitor_cam_id = i;
			break;
		}
	}
	fileIO->start2DVideoReader();

	nose_pos.resize(8, btVector3(0.0, 0.0, 0.0));
	nose_pos_enable.resize(8, false);
}

void loopApp()
{
	int i, j, k, step;

	if (online_mode)
	{
		double ts_prev = ts_online;

		if (recording)
		{
			boost::thread_group thr_record;
			thr_record.create_thread(boost::bind(&MyFileIO::writeMergedPcFrame, fileIO, pc_crntframe, ts_online));
			for (i = 0; i < fileIO->metadata.num_camera; i++)
			{
				cv::Mat color;
				fileIO->read2DVideoFrame(color, i, 0);
				if (fileIO->metadata.rec_save_2d_vid[i]) thr_record.create_thread(boost::bind(&MyFileIO::write2DVideoFrame, fileIO, color, i));
			}
			thr_record.join_all();
		}

		boost::thread thr_readframe(boost::bind(&readFrame, 0, false));
		while (!thr_readframe.timed_join(boost::posix_time::milliseconds(0)))
		{
			tracker->runSimStep();
		}
		
		if (recording)
		{
			fprintf(fp_online_result, "%d, %lf, ", rec_frame_cnt, (ts_prev - rec_ts0) / 1000.0);

			for (i = 0; i < tracker->sm.size(); i++)
			{
				if (checkModelInRoi(i)) fprintf(fp_online_result, "1, ");
				else fprintf(fp_online_result, "0, ");
			}
			for (i = 0; i < tracker->sm.size(); i++)
			{
				RodentTracker::SkeletonModelPos pos;
				tracker->getModelPos(i, pos);
				writeCenterInCsv(pos.hd, fp_online_result);
				writeCenterInCsv(pos.n, fp_online_result);
				writeCenterInCsv(pos.t, fp_online_result);
				writeCenterInCsv(pos.hp, fp_online_result);
			}
			fprintf(fp_online_result, "\n");

			rec_frame_cnt++;
		}
		
		tracker->setPointCloud(pc_crntframe);
		updateColorVideoFrame();
		glutPostRedisplay();
	}
	else
	{
		if (preproc_finished)
		{

			if (i_preframe != i_crntframe)
			{
				readFrame(i_crntframe);
				i_preframe = i_crntframe;

				updateColorVideoFrame();
			}

			if (i_crntframe <= tracker_result.last_frame)
			{
				for (i = 0; i < tracker->sm.size(); i++) tracker->setModelPos(i, tracker_result.pos[i][i_crntframe]);
			}

			if (!vplay)
			{
				tracker->runSimStep();
			}
			else if (vplay && tracker_result.last_frame < i_crntframe)
			{
				tracker->runSimUntilSteadyState();

				for (i = 0; i < tracker->sm.size(); i++)
				{
					tracker->getModelPos(i, tracker_result.pos[i][i_crntframe]);

					tracker_result.sm_enabled[i][i_crntframe] = checkModelInRoi(i);
				}
				tracker_result.last_frame = i_crntframe;
			}

			if (vplay && !online_mode)
			{
				static clock_t t_pre = clock();

				int next_frame = (i_crntframe + 1) % fileIO->getNumFrame();

				if (fileIO->getTimestamp(next_frame) - fileIO->getTimestamp(i_crntframe) < float(clock() - t_pre) * play_speed)
				{
					i_crntframe = next_frame;
					t_pre = clock();
				}

			}

			// stop playing at the last frame
			if (!online_mode && i_crntframe == fileIO->getNumFrame() - 1) vplay = false;

		}

		if (vplay && disp_every_5th_frame && i_crntframe % 5 == 0) glutPostRedisplay();
		else if (vplay && !disp_every_5th_frame) glutPostRedisplay();
		else if (!vplay) glutPostRedisplay();
	}

}

void drawGUI()
{
	int i;

	static bool show_player_window = preproc_finished;
	static bool show_modeledit_window = false;
	static bool show_phyparam_window = false;
	static bool show_roi_setting_window = false;
	static bool show_color_filter_window = false;
	static bool show_colorvideo_monitor = false;
	static bool show_tracking_tools_window = false;
	static bool show_view_setting_window = false;
	static bool show_framerate_window = false;
	static bool show_measurement_window = false;
	static bool show_outlier_removal_filter_window = false;
	static bool show_online_mode_control_window = online_mode;

	ImGui_ImplGLUT_NewFrame(getAppScreenWidth(), getAppScreenHeight());

	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (!online_mode && ImGui::MenuItem("load result"))
			{
				if (fileIO->checkTrackingResultExist())
				{
					fileIO->loadMetadata();
					fileIO->loadTrackingParam(*tracker_param);

					tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
					tracker->setPointCloud(pc_crntframe);
					initTrackingResult();

					fileIO->loadTrackingResult(tracker_result);

					readFrame(i_crntframe);
				}
				else
				{
					std::cout << "no result file exist." << std::endl;
				}
			}

			if (!online_mode && ImGui::MenuItem("Save result"))
			{
				fileIO->saveMetadata();
				fileIO->saveTrackingParam(*tracker_param);

				fileIO->saveTrackingResult(tracker_result);
			}

			if (!online_mode && ImGui::MenuItem("Export result"))
			{
				OPENFILENAMEA ofn;
				char szFile[MAX_PATH];
				std::string session_name;
				fileIO->getSessionName(session_name);
				sprintf(szFile, "%s.result.csv", session_name.c_str());
				ZeroMemory(&ofn, sizeof(ofn));
				ofn.lStructSize = sizeof(OPENFILENAMEA);
				ofn.lpstrFilter = "Tracking Result (*.csv)\0*.csv\0";
				ofn.lpstrFile = szFile;
				ofn.nMaxFile = MAX_PATH;
				ofn.Flags = OFN_OVERWRITEPROMPT;
				ofn.lpstrDefExt = "csv";
				ofn.lpstrTitle = "Save";

				GetSaveFileNameA(&ofn);

				std::vector<float> ts;

				for (i = 0; i < fileIO->getNumFrame(); i++) ts.push_back(fileIO->getTimestamp(i));

				tracker_result.exportAsCsv(ts, ofn.lpstrFile);
			}

			if (!online_mode) ImGui::Separator();

			if (ImGui::MenuItem("Import Params"))
			{
				OPENFILENAMEA ofn;
				char szFile[MAX_PATH] = "";
				ZeroMemory(&ofn, sizeof(ofn));
				ofn.lStructSize = sizeof(OPENFILENAMEA);
				ofn.lpstrFilter = "Tracking Paramaters (*.xml)\0*.xml\0";
				ofn.lpstrFile = szFile;
				ofn.nMaxFile = MAX_PATH;
				ofn.Flags = OFN_FILEMUSTEXIST;
				ofn.lpstrTitle = "Load";

				GetOpenFileNameA(&ofn);

				std::string fullpath(ofn.lpstrFile);

				tracker_param->load(fullpath.c_str());

				tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
				tracker->setPointCloud(pc_crntframe);
				initTrackingResult();
				initModelPos();

				readFrame(i_crntframe);
			}

			if (ImGui::MenuItem("Export Params"))
			{
				OPENFILENAMEA ofn;
				char szFile[MAX_PATH] = "params.xml";
				ZeroMemory(&ofn, sizeof(ofn));
				ofn.lStructSize = sizeof(OPENFILENAMEA);
				ofn.lpstrFilter = "Tracking Paramaters (*.xml)\0*.xml\0";
				ofn.lpstrFile = szFile;
				ofn.nMaxFile = MAX_PATH;
				ofn.Flags = OFN_OVERWRITEPROMPT;
				ofn.lpstrDefExt = "xml";
				ofn.lpstrTitle = "Save";

				GetSaveFileNameA(&ofn);

				std::string fullpath(ofn.lpstrFile);

				tracker_param->save(fullpath.c_str());
			}

			if (ImGui::MenuItem("Reset Params to default"))
			{
				tracker_param = std::shared_ptr<RodentTrackerParam>(new RodentTrackerParam());
				tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
				tracker->setPointCloud(pc_crntframe);
				initTrackingResult();
				initModelPos();

				readFrame(i_crntframe);
			}

			ImGui::Separator();

			if (ImGui::MenuItem("Quit", "Alt+F4")) { glutLeaveMainLoop(); }

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Window"))
		{
			if (!online_mode && ImGui::MenuItem("Player", "", &show_player_window)) {}
			if (online_mode && ImGui::MenuItem("Online mode control", "", &show_online_mode_control_window)) {}
			if (ImGui::MenuItem("Tracking tools", "", &show_tracking_tools_window)) {}
			if (ImGui::MenuItem("Color video monitor", "", &show_colorvideo_monitor)) {}
			if (ImGui::MenuItem("View setting", "", &show_view_setting_window)) {}

			ImGui::Separator();

			if (ImGui::MenuItem("Model editor", "", &show_modeledit_window)) {}
			if (ImGui::MenuItem("Physics sim params", "", &show_phyparam_window)) {}
			if (ImGui::MenuItem("ROI setting", "", &show_roi_setting_window)) {}
			if (ImGui::MenuItem("Color filter", "", &show_color_filter_window)) {}
			if (ImGui::MenuItem("Outlier removal", "", &show_outlier_removal_filter_window)) {}

			ImGui::Separator();

			if (ImGui::MenuItem("Measurement", "", &show_measurement_window)) {}

			ImGui::Separator();

			if (ImGui::MenuItem("Frame rate", "", &show_framerate_window)) {}

			ImGui::EndMenu();
		}

		
		if (ImGui::BeginMenu("Special"))
		{
			if (ImGui::MenuItem("Export nose positions"))
			{
				exportNosePos();
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}

	if (!online_mode && show_player_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Player", &show_player_window, ImGuiWindowFlags_AlwaysAutoResize);
		
		ImGui::PushItemWidth(500);

		std::string str = std::to_string((fileIO->getTimestamp(i_crntframe) - fileIO->getTimestamp(0)) / 1000.0) + "sec###CurrentFrame";
		ImGui::SliderInt(str.c_str(), &i_crntframe, 0, fileIO->getNumFrame() - 1, "Frame: %.0f");
		
		ImGui::Checkbox("Play    ", &vplay);

		ImGui::SameLine();
		if (ImGui::Button(" << ##Player")) {i_crntframe -= 10;}

		ImGui::SameLine();
		if (ImGui::Button(" < ##Player")) {i_crntframe--; }

		ImGui::SameLine();
		if (ImGui::Button(" > ##Player")) {i_crntframe++;}

		ImGui::SameLine();
		if (ImGui::Button(" >> ##Player")) {i_crntframe += 10;}

		ImGui::SameLine();

		str = "Tracking has been completed until frame no. " + std::to_string(tracker_result.last_frame);
		ImGui::Text(str.c_str());

		ImGui::PopItemWidth();

		// optional functions
		if (ImGui::TreeNode("Options##Player"))
		{
			ImGui::PushItemWidth(400);

			ImGui::Checkbox("Display every 5th frame during playing", &disp_every_5th_frame); 

			ImGui::SliderFloat("Play speed##Player", &play_speed, 0.1, 32.0, "%.1f", 3.0);

			ImGui::PopItemWidth();

			ImGui::TreePop();
		}
		// correct crnt_frame 
		if (i_crntframe < 0) i_crntframe = 0;
		if (i_crntframe >  fileIO->getNumFrame() - 1) i_crntframe = fileIO->getNumFrame() - 1;

		ImGui::End();
	}

	if (online_mode && show_online_mode_control_window)
	{
		static char rec_session_name[256] = "untitled_session";

		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Online Mode Control", &show_online_mode_control_window, ImGuiWindowFlags_AlwaysAutoResize);

		if (!recording)
		{
			ImGui::InputText("Session name", rec_session_name, 256);

			if (ImGui::Button("Start Recording##Online control"))
			{
				char dname[1024];
				sprintf(dname, "data\\%s\\", rec_session_name);
				fileIO->resetRecDir(dname, rec_session_name);
				startRecording();
			}
		}
		else
		{
			ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(180, 0, 0));
			char buf[256];
			sprintf(buf, "REC - %s", rec_session_name);
			ImGui::InputText("Session name", buf, 256);
			ImGui::PopStyleColor();

			if (ImGui::Button("Stop Recording##Online control"))
			{
				stopRecording();
			}
		}
		ImGui::End();

	}

	if (show_modeledit_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Model Editor", &show_modeledit_window, ImGuiWindowFlags_AlwaysAutoResize);

		if (ImGui::SliderInt("Number of Animals##ModelEditor", &tracker_param->n_animal, 1, 6)) { tracker_param->setNumAnimal(tracker_param->n_animal); }

		for (i = 0; i < tracker_param->n_animal; i++)
		{
			std::string str = "Model " + std::to_string(i) + "##ModelEditor";
			ImGui::SliderFloat(str.c_str(), &tracker_param->animal_size[i], 50.0, 200.0, "%.0f %%");
		}

		if (ImGui::TreeNode("Backbone Length##ModelEditor"))
		{
			ImGui::SliderFloat("Trunk-Neck##ModelEditor", &tracker_param->standard_model.t_n_len, 1.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Trunk-Hip##ModelEditor", &tracker_param->standard_model.t_hp_len, 1.0, 200.0, "%.1f mm");

			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Radius for Physical Body##ModelEditor"))
		{
			ImGui::SliderFloat("Head##RB_ModelEditor", &tracker_param->standard_model.hd_rbody, 1.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Neck##RB_ModelEditor", &tracker_param->standard_model.n_rbody, 1.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Trunk##RB_ModelEditor", &tracker_param->standard_model.t_rbody, 1.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Hip##RB_ModelEditor", &tracker_param->standard_model.hp_rbody, 1.0, 200.0, "%.1f mm");

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Radius for Attraction##ModelEditor"))
		{
			ImGui::SliderFloat("Head##RA_ModelEditor", &tracker_param->standard_model.hd_ratt, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Neck_in##RA_ModelEditor", &tracker_param->standard_model.n_ratt1, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Neck_out##RA_ModelEditor", &tracker_param->standard_model.n_ratt2, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Trunk##RA_ModelEditor", &tracker_param->standard_model.t_ratt, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Hip##RA_ModelEditor", &tracker_param->standard_model.hp_ratt, 0.0, 200.0, "%.1f mm");

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Radius for Repulsion##ModelEditor"))
		{
			ImGui::SliderFloat("Head##RR_ModelEditor", &tracker_param->standard_model.hd_rrep, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Neck##RR_ModelEditor", &tracker_param->standard_model.n_rrep, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Trunk##RR_ModelEditor", &tracker_param->standard_model.t_rrep, 0.0, 200.0, "%.1f mm");
			ImGui::SliderFloat("Hip##RR_ModelEditor", &tracker_param->standard_model.hp_rrep, 0.0, 200.0, "%.1f mm");

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Neck-Head Joint##ModelEditor"))
		{
			ImGui::SliderFloat("Left##NHJ_ModelEditor", &tracker_param->standard_model.n_hd_angl, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Right##NHJ_ModelEditor", &tracker_param->standard_model.n_hd_angr, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Dorsal##NHJ_ModelEditor", &tracker_param->standard_model.n_hd_angd, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Ventral##NHJ_ModelEditor", &tracker_param->standard_model.n_hd_angv, 0.0, 180.0, "%.0f deg");

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Trunk-Neck Joint##ModelEditor"))
		{
			ImGui::SliderFloat("Left##TNJ_ModelEditor", &tracker_param->standard_model.t_n_angl, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Right##TNJ_ModelEditor", &tracker_param->standard_model.t_n_angr, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Dorsal##TNJ_ModelEditor", &tracker_param->standard_model.t_n_angd, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Ventral##TNJ_ModelEditor", &tracker_param->standard_model.t_n_angv, 0.0, 180.0, "%.0f deg");

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Trunk-Hip Joint##ModelEditor"))
		{
			ImGui::SliderFloat("Anterior##THJ_ModelEditor", &tracker_param->standard_model.hp_t_anga, 0.0, 180.0, "%.0f deg");
			ImGui::SliderFloat("Posterior##THJ_ModelEditor", &tracker_param->standard_model.hp_t_angp, 0.0, 180.0, "%.0f deg");

			ImGui::TreePop();
		}

		if(ImGui::Button("Apply##ModelEditor")) 
		{
			tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
			tracker->setPointCloud(pc_crntframe);
			initTrackingResult();
			initModelPos();
		}

		ImGui::End();
	}
	
	if (show_phyparam_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Physics Sim Params", &show_phyparam_window, ImGuiWindowFlags_AlwaysAutoResize);

		ImGui::SliderFloat("delta-T##PhysicsParams", &tracker_param->sim_dt, 0.01, 0.01, "%.3f sec");
		ImGui::SliderFloat("Attraction Constant##PhysicsParams", &tracker_param->f_att, 0.01, 1.0, "%.2f Ns/m");
		ImGui::SliderFloat("Repulsion Constant##PhysicsParams", &tracker_param->f_rep, 0.01, 1.0, "%.2f Ns/m");
		ImGui::SliderFloat("Threshold for Steady-state##PhysicsParams", &tracker_param->v_term, 0.001, 0.1, "%.3f m/s", 3.0);
		ImGui::SliderFloat("Rotation Correction Constant##PhysicsParams", &tracker_param->f_const3, 0.001, 1.0, "%.3f Ns", 3.0);
		ImGui::SliderFloat("Floor-Hip Attraction Constant##PhysicsParams", &tracker_param->f_hip2floor, 0.0, 1.0, "%.2f");
		ImGui::SliderFloat("Floor Height##PhysicsParams", &tracker_param->h_floor, -1.0, 1.0, "%.2f m");

		if (ImGui::Button("Apply##PhysicsParams"))
		{
			tracker = std::shared_ptr<RodentTracker>(new RodentTracker(tracker_param));
			tracker->setPointCloud(pc_crntframe);
			initTrackingResult();
			initModelPos();
		}

		ImGui::End();
	}
	
	if (show_roi_setting_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("ROI setting", &show_roi_setting_window, ImGuiWindowFlags_AlwaysAutoResize);

		if (ImGui::DragFloatRange2("X (m)##ROI", &fileIO->metadata.roi.x[0], &fileIO->metadata.roi.x[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f")) { readFrame(i_crntframe); }
		if (ImGui::DragFloatRange2("Y (m)##ROI", &fileIO->metadata.roi.y[0], &fileIO->metadata.roi.y[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f")) { readFrame(i_crntframe); }
		if (ImGui::DragFloatRange2("Z (m)##ROI", &fileIO->metadata.roi.z[0], &fileIO->metadata.roi.z[1], 0.01f, -10.0f, 10.0f, "Min: %.2f", "Max: %.2f")) { readFrame(i_crntframe); }

		if (ImGui::Button("Reset##ROI"))
		{
			fileIO->loadMetadata();

			readFrame(i_crntframe);
		}

		ImGui::End();
	}

	if (show_color_filter_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Color Filter", &show_color_filter_window, ImGuiWindowFlags_AlwaysAutoResize);

		if (ImGui::Checkbox("Enable Filtering##Color Filt", &tracker_param->cf_enable)) { readFrame(i_crntframe); }

		if (ImGui::DragIntRange2("H##Color Filt", &tracker_param->cf_hsv_min[0], &tracker_param->cf_hsv_max[0], 1, 0, 255, "Min: %.0f", "Max: %.0f")) { readFrame(i_crntframe); }
		if (ImGui::DragIntRange2("S##Color Filt", &tracker_param->cf_hsv_min[1], &tracker_param->cf_hsv_max[1], 1, 0, 255, "Min: %.0f", "Max: %.0f")) { readFrame(i_crntframe); }
		if (ImGui::DragIntRange2("V##Color Filt", &tracker_param->cf_hsv_min[2], &tracker_param->cf_hsv_max[2], 1, 0, 255, "Min: %.0f", "Max: %.0f")) { readFrame(i_crntframe); }

		if (ImGui::RadioButton("Exclude points in the range", &tracker_param->cf_inc, 0)) { readFrame(i_crntframe); }
		if (ImGui::RadioButton("Include points in the range", &tracker_param->cf_inc, 1)) { readFrame(i_crntframe); }

		ImGui::End();
	}

	if (show_colorvideo_monitor)
	{
		ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiSetCond_Once);
		ImGui::Begin("2D Video Monitor", &show_colorvideo_monitor);

		int w = ImGui::GetWindowWidth() - 15;
		ImGui::Image((void*)(GLuint)colorvideo_monitor_tex.getTexId(), ImVec2(w, w * 3 / 4));

		ImGui::Text("Camera:"); ImGui::SameLine();

		for (i = 0; i < fileIO->metadata.num_camera; i++)
		{
			if (i == 0 || i % 4 != 0) ImGui::SameLine();

			if (!colorvideo_exist[i]) continue;
			char buf[256];
			sprintf(buf, "%d##2DVideoMonitor", i + 1);
			if (ImGui::RadioButton(buf, &colorvideo_monitor_cam_id, i)) updateColorVideoFrame();
		}

		if (ImGui::Button("Use the camera view##2DVideoMonitor"))
		{
			pcl::PointCloud<pcl::PointXYZ> pc_camera_pos;
			pcl::PointXYZ p_o(0.0, 0.0, 0.0);
			pcl::PointXYZ p_z(0.0, 0.0, 1.0);
			pcl::PointXYZ p_y(0.0, 1.0, 0.0);
			pc_camera_pos.push_back(p_o);
			pc_camera_pos.push_back(p_z);
			pc_camera_pos.push_back(p_y);

			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.pc_transforms[colorvideo_monitor_cam_id]);
			pcl::transformPointCloud(pc_camera_pos, pc_camera_pos, fileIO->metadata.ref_cam_transform);

			Eigen::Vector3f v_roi_center(fileIO->metadata.roi.x.mean(), fileIO->metadata.roi.y.mean(), fileIO->metadata.roi.z.mean());
			Eigen::Vector3f v_o(pc_camera_pos[0].x, pc_camera_pos[0].y, pc_camera_pos[0].z);
			Eigen::Vector3f v_z(pc_camera_pos[1].x, pc_camera_pos[1].y, pc_camera_pos[1].z);
			Eigen::Vector3f v_y(pc_camera_pos[2].x, pc_camera_pos[2].y, pc_camera_pos[2].z);

			float d = (v_roi_center - v_o).dot((v_z - v_o).normalized());
			Eigen::Vector3f v_view = v_o + d * (v_z - v_o).normalized();

			Eigen::Matrix4f trans = fileIO->metadata.ref_cam_transform.matrix() * fileIO->metadata.pc_transforms[colorvideo_monitor_cam_id];
			trans = trans.inverse();
			float roll = atan2f(-trans(0, 1), trans(1, 1));

			resetView(v_o.x(), v_o.y(), v_o.z(), v_view.x(), v_view.y(), v_view.z(), roll);
			
		}
		
		ImGui::End();
	}

	if (show_tracking_tools_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Tracking tools", &show_tracking_tools_window, ImGuiWindowFlags_AlwaysAutoResize);

		for (i = 0; i < tracker_param->n_animal; i++)
		{
			char label[64];
			float clr[4];

			getColor(i, clr);

			sprintf(label, "Model %d: ", i);
			ImGui::Text(label);

			ImGui::PushStyleColor(ImGuiCol_Button, ImColor::ImColor(clr[0] * 0.5f, clr[1] * 0.5f, clr[2] * 0.5f));

			ImGui::SameLine();
			sprintf(label, "Move##Tracking Tools %d", i);
			if (ImGui::Button(label))
			{
				if (pos_cursor.size() == 2)
				{
					float pitch, yaw;

					Eigen::Vector3f p1, p2;

					p1 << pos_cursor[0].x, pos_cursor[0].y, pos_cursor[0].z;
					p2 << pos_cursor[1].x, pos_cursor[1].y, pos_cursor[1].z;

					Eigen::Vector3f d = p2 - p1;

					yaw = acos(d.x() / sqrt(d.x()*d.x() + d.z()*d.z()));
					if (d.z() < 0) yaw = -yaw;
					pitch = atan(d.y() / sqrt(d.x()*d.x() + d.z()*d.z()));

					tracker->setModelPos(i, p1.x() + d.x() / 2, p1.y() + d.y() / 2, p1.z() + d.z() / 2, pitch, -yaw);

					pos_cursor.clear();

					if (tracker_result.last_frame >= i_crntframe) tracker_result.last_frame = i_crntframe - 1;
				}
			}

			ImGui::SameLine();
			sprintf(label, "Withdraw##Tracking Tools %d", i);
			if (ImGui::Button(label))
			{
				initModelPos(i, 1.0);
				if (tracker_result.last_frame >= i_crntframe) tracker_result.last_frame = i_crntframe - 1;
			}

			ImGui::PopStyleColor();

			if (i != tracker_param->n_animal - 1) ImGui::Separator();
		}

		ImGui::End();
	}

	if (show_view_setting_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("View setting", &show_view_setting_window, ImGuiWindowFlags_AlwaysAutoResize);

		ImGui::Checkbox("Show Model##View setting", &view_show_model);
		ImGui::Checkbox("Show Nose##View setting", &view_show_nose);
		ImGui::Checkbox("Show Axis##View setting", &view_show_axis);

		ImGui::SliderFloat("Point Size##View setting", &view_pointsize, 1.0, 20.0, "%.1f");
		ImGui::SliderFloat("Point Transparency##View setting", &view_transparency, 0.0, 1.0, "%.2f");
		
		ImGui::Text("Model Coloring:"); 
		ImGui::SameLine(); 
		ImGui::RadioButton("Normal##Model Coloring", &view_model_coloring, 0); 
		ImGui::SameLine(); 
		ImGui::RadioButton("Whiter Head##Model Coloring", &view_model_coloring, 1);
		ImGui::SameLine(); 
		ImGui::RadioButton("Darker Body##Model Coloring", &view_model_coloring, 2);

		ImGui::End();
	}

	if (!preproc_finished)
	{
		ImGui::OpenPopup("Preprocessing");
		if (ImGui::BeginPopupModal("Preprocessing", NULL, ImGuiWindowFlags_AlwaysAutoResize))
		{
			static int voxel_size = 10;

			ImGui::Text("Preprocessing has not been finished.");

			ImGui::SliderInt("Voxel size (mm)##PreProc", &voxel_size, 1, 10);

			if (ImGui::Button("Start Preprocessing##PreProc"))
			{
				fileIO->preprosessData((float)voxel_size / 1000.0);
				preproc_finished = true;
				initTracking();

				show_player_window = true;
			}

			ImGui::EndPopup();
		}
	}

	if (show_framerate_window)
	{
		ImGui::SetNextWindowPos(ImVec2(getAppScreenWidth() - 130, getAppScreenHeight() - 40));
		ImGui::Begin("Frame Rate Window", nullptr, ImVec2(0, 0), 0.3f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
		ImGui::Text("%.1f frames/sec", ImGui::GetIO().Framerate);
		ImGui::End();
	}

	if (show_measurement_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Measurement", &show_measurement_window, ImGuiWindowFlags_AlwaysAutoResize);

		static char measurement_output[1024 * 16] = "";

		if (ImGui::TreeNode("Location##Measurement"))
		{

			ImGui::Text("Select a point with 'Shift + left click'");
			if (ImGui::Button("Measure##Location")) 
			{
				if (pos_cursor.size() == 1)
				{
					sprintf(measurement_output, "%slocation (x, y, z; m): \n  %.3f, %.3f, %.3f\n",
						measurement_output, pos_cursor[0].x, pos_cursor[0].y, pos_cursor[0].z);
				}
				else
				{
					sprintf(measurement_output, "%slocation (x, y, z; m): \n  Error! Select a point.\n", measurement_output);
				}
			}

			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Distance##Measurement"))
		{
			ImGui::Text("Select two points with 'Shift + left click'");
			if (ImGui::Button("Measure##Distance"))
			{
				if (pos_cursor.size() == 2)
				{
					float dx = fabs(pos_cursor[0].x - pos_cursor[1].x);
					float dy = fabs(pos_cursor[0].y - pos_cursor[1].y);
					float dz = fabs(pos_cursor[0].z - pos_cursor[1].z);
					float D = sqrt(dx*dx + dy*dy + dz*dz);

					sprintf(measurement_output, "%sDistance (m): \n  dX = %.3f\n  dY= %.3f\n  dZ = %.3f\n  distance = %.3f\n",
						measurement_output, dx, dy, dz, D);
				}
				else
				{
					sprintf(measurement_output, "%sDistance (m): \n  Error! Select two points.\n", measurement_output);
				}
			}

			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Cam xyz to world xyz##Measurement"))
		{
			static float xyz_cam[3] = { 0.0, 0.0, 0.0 };
			ImGui::InputFloat("X (m)", &xyz_cam[0]);
			ImGui::InputFloat("Y (m)", &xyz_cam[1]);
			ImGui::InputFloat("Z (m)", &xyz_cam[2]);

			if (ImGui::Button("Calculate##Cam2World"))
			{
				sprintf(measurement_output, "%sXYZ in world coordinate (m):\n", measurement_output);
				for (i = 0; i < fileIO->metadata.num_camera; i++)
				{
					pcl::PointCloud<pcl::PointXYZ> pc_pos_in_world;
					pcl::PointXYZ p(xyz_cam[0], xyz_cam[1], xyz_cam[2]);	
					pc_pos_in_world.push_back(p);

					pcl::transformPointCloud(pc_pos_in_world, pc_pos_in_world, fileIO->metadata.pc_transforms[i]);
					pcl::transformPointCloud(pc_pos_in_world, pc_pos_in_world, fileIO->metadata.ref_cam_transform);

					sprintf(measurement_output, "%s  cam%02d: %.3f, %.3f, %.3f\n", measurement_output, i+1, pc_pos_in_world[0].x, pc_pos_in_world[0].y, pc_pos_in_world[0].z);
				}
			}

			ImGui::TreePop();
		}

		ImGui::Separator();
		ImGui::Text("Output:");
		ImGui::InputTextMultiline("##source", measurement_output, 1024 * 16,
			ImVec2(350, ImGui::GetTextLineHeight() * 8), 
			ImGuiInputTextFlags_AllowTabInput | ImGuiInputTextFlags_ReadOnly);

		if (ImGui::Button("Clear##Measurement"))
		{
			sprintf(measurement_output, "\0");
		}

		ImGui::End();
	}

	if (show_outlier_removal_filter_window)
	{
		ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiSetCond_Once);
		ImGui::Begin("Outlier Removal", &show_outlier_removal_filter_window, ImGuiWindowFlags_AlwaysAutoResize);

		if (ImGui::Checkbox("Enable Filtering##OR Filt", &tracker_param->orf_enable)) { readFrame(i_crntframe); }
		if (ImGui::SliderInt("Min Neighbors##OR Filt", &tracker_param->orf_meanK, 1, 200)) { readFrame(i_crntframe); }
		if (ImGui::SliderFloat("Radius (m)##OR Filt", &tracker_param->orf_thresh, 0.001, 1.0, "%.3f", 3.0)) { readFrame(i_crntframe); }
	
		ImGui::End();
	}

	if (online_mode)
	{
		ImGui::SetNextWindowPos(ImVec2(5, getAppScreenHeight() - 40));
		ImGui::Begin("Online Mode Indication", nullptr, ImVec2(0, 0), 0.3f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
		ImGui::Text("ONLINE MODE");
		ImGui::End();
	}

	mouseAndKey();

	ImGui::Render();

}

void displayApp()
{
	if (preproc_finished)
	{
		showModel(false);
		showNose();
		showSetPosCursor();
		showPC();
	}

	glDisable(GL_LIGHTING);

	if (view_show_axis) drawAxis(0.2);

	drawROI(fileIO->metadata.roi);

	drawGUI();
}



