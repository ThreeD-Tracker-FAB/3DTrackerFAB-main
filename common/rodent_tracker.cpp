#include "rodent_tracker.h"

#include <corecrt_math_defines.h>
#include <list>

RodentTrackerParam::RodentTrackerParam()
{
	// initialize with defauld values
	sim_dt = 0.01;				
	f_att = 0.2;				
	f_rep = 0.1;				
	v_term = 0.025;				
	f_const3 = 0.005;		
	f_hip2floor = 0.05;
	h_floor = 0.0;

	standard_model.t_n_len = 80.0;
	standard_model.t_hp_len = 50.0;

	standard_model.hd_rbody = 30.0;
	standard_model.n_rbody = 10.0;
	standard_model.t_rbody = 30.0;
	standard_model.hp_rbody = 25.0;

	standard_model.hd_ratt = 40.0;
	standard_model.n_ratt1 = 40.0;
	standard_model.n_ratt2 = 65.0;
	standard_model.t_ratt = 60.0;
	standard_model.hp_ratt = 50.0;

	standard_model.hd_rrep = 10.0;
	standard_model.n_rrep = 10.0;
	standard_model.t_rrep = 15.0;
	standard_model.hp_rrep = 20.0;

	standard_model.n_hd_angl = 45.0;
	standard_model.n_hd_angr = 45.0;
	standard_model.n_hd_angd = 60.0;
	standard_model.n_hd_angv = 60.0;

	standard_model.t_n_angl = 30.0;
	standard_model.t_n_angr = 30.0;
	standard_model.t_n_angd = 20.0;
	standard_model.t_n_angv = 30.0;

	standard_model.hp_t_anga = 10.0;
	standard_model.hp_t_angp = 60.0;

	setNumAnimal(2);

	cf_hsv_min[0] = 0; cf_hsv_min[1] = 0; cf_hsv_min[2] = 0;
	cf_hsv_max[0] = 255; cf_hsv_max[1] = 255; cf_hsv_max[2] = 255;
	cf_enable = false;
	cf_inc = 0;

	orf_enable = false;
	orf_meanK = 10;
	orf_thresh = 0.05;
}

void RodentTrackerParam::save(const char * filename)
{
	std::ofstream ofs(filename);
	boost::archive::xml_oarchive oa(ofs);
	oa << BOOST_SERIALIZATION_NVP(this);
	ofs.close();
}

int RodentTrackerParam::load(const char * filename)
{
	std::ifstream ifs(filename);
	if (!ifs)
	{
		std::cout << "can not open file:" << filename << std::endl;
		return 0;
	}

	boost::archive::xml_iarchive ia(ifs);
	ia >> BOOST_SERIALIZATION_NVP(*this);
	ifs.close();

	return 1;
}

void RodentTrackerParam::setNumAnimal(int n)
{
	n_animal = n;
	animal_size.clear();

	for (int i = 0; i < n_animal; i++)
	{
		animal_size.push_back(100.0);
	}
}

RodentTracker::RodentTracker(std::shared_ptr<RodentTrackerParam> p)
{
	params = p;

	initialize();
}

void RodentTracker::setModelPos(int id, float x, float y, float z, float pitch, float yaw)
{
	btTransform ts;
	btQuaternion quat;

	ts.setIdentity();
	ts.setOrigin(btVector3(x, y, z));
	quat.setEuler(yaw, 0, pitch);
	ts.setRotation(quat);

	sm[id].body_hd->setCenterOfMassTransform(ts * sm_pos0.hd);
	sm[id].body_n->setCenterOfMassTransform(ts * sm_pos0.n);
	sm[id].body_t->setCenterOfMassTransform(ts * sm_pos0.t);
	sm[id].body_hp->setCenterOfMassTransform(ts * sm_pos0.hp);
}

void RodentTracker::setModelPos(int id, SkeletonModelPos & sm_pos)
{
	sm[id].body_hd->setCenterOfMassTransform(sm_pos.hd);
	sm[id].body_n->setCenterOfMassTransform(sm_pos.n);
	sm[id].body_t->setCenterOfMassTransform(sm_pos.t);
	sm[id].body_hp->setCenterOfMassTransform(sm_pos.hp);
}

void RodentTracker::getModelPos(int id, SkeletonModelPos & sm_pos)
{
	sm_pos.hd = sm[id].body_hd->getCenterOfMassTransform();
	sm_pos.n = sm[id].body_n->getCenterOfMassTransform();
	sm_pos.t = sm[id].body_t->getCenterOfMassTransform();
	sm_pos.hp = sm[id].body_hp->getCenterOfMassTransform();
}

void RodentTracker::runSimStep()
{
	applyForce();
	physim->stepSimulation(params->sim_dt, 1, params->sim_dt);
}

void RodentTracker::runSimUntilSteadyState()
{
	int i, j, k, step;

	std::vector< std::vector < std::list< btVector3 > > > posHist;		// recennt history of positions of body parts

	posHist.resize(sm.size());
	for (i = 0; i<sm.size(); i++)
	{
		posHist[i].resize(4);
		for (j = 0; j<4; j++) posHist[i][j].clear();
	}

	for (step = 0; step < 1000; step++)
	{
		runSimStep();

		for (i = 0; i<sm.size(); i++)
		{
			posHist[i][0].push_back(sm[i].body_hd->getCenterOfMassPosition());
			posHist[i][1].push_back(sm[i].body_n->getCenterOfMassPosition());
			posHist[i][2].push_back(sm[i].body_t->getCenterOfMassPosition());
			posHist[i][3].push_back(sm[i].body_hp->getCenterOfMassPosition());


			if (posHist[i][0].size() > 8)
			{
				posHist[i][0].pop_front();
				posHist[i][1].pop_front();
				posHist[i][2].pop_front();
				posHist[i][3].pop_front();
			}
		}

		if (posHist[0][0].size() == 8)
		{
			// calculate the maximum velocity among those of body parts
			float max_d = 0.0;
			for (i = 0; i<sm.size(); i++)
			{
				for (j = 0; j<4; j++)
				{
					btVector3 P_pre, P;
					std::list< btVector3 >::iterator it = posHist[i][j].begin();

					P_pre.setValue(0, 0, 0);
					for (k = 0; k<4; k++)
					{
						P_pre += *it;
						it++;
					}
					P_pre /= 4.0;

					P.setValue(0, 0, 0);
					for (k = 0; k<4; k++)
					{
						P += *it;
						it++;
					}
					P /= 4.0;

					float d = P.distance(P_pre);
					if (d > max_d) max_d = d;
				}
			}
			// terminate if system reach steady-state
			if (max_d / (params->sim_dt*4.0) < params->v_term)
			{
				if (correctUpsideDown() == -1) break;
			}
		}
	}
}

void RodentTracker::setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& pc)
{
	pc_to_fit = pc;
}

void RodentTracker::togglePointForce(int animal_id, bool state)
{
	point_force_active[animal_id] = state;
}

void RodentTracker::applyForce()
{
	int id;

	updateDistMat();

	updatePointAttribution();

	float R2;
	btVector3 v;
	for (id = 0; id<sm.size(); id++)
	{
		//Clear Forces and Velocity
		sm[id].body_hd->clearForces();
		sm[id].body_hd->setAngularVelocity(btVector3(0, 0, 0));
		sm[id].body_hd->setLinearVelocity(btVector3(0, 0, 0));

		sm[id].body_n->clearForces();
		sm[id].body_n->setAngularVelocity(btVector3(0, 0, 0));
		sm[id].body_n->setLinearVelocity(btVector3(0, 0, 0));

		sm[id].body_t->clearForces();
		sm[id].body_t->setAngularVelocity(btVector3(0, 0, 0));
		sm[id].body_t->setLinearVelocity(btVector3(0, 0, 0));

		sm[id].body_hp->clearForces();
		sm[id].body_hp->setAngularVelocity(btVector3(0, 0, 0));
		sm[id].body_hp->setLinearVelocity(btVector3(0, 0, 0));

		
		if (point_force_active[id])
		{
			//apply Attraction force (f_a)

			R2 = powf(sm[id].params.hd_ratt, 2.0);
			v = sm[id].body_hd->getCenterOfMassPosition();
			sm[id].body_hd->applyCentralImpulse(params->f_att * calcAttraction(R2, v, id, 1));

			R2 = powf(sm[id].params.n_ratt2, 2.0);
			v = sm[id].body_n->getCenterOfMassPosition();
			sm[id].body_n->applyCentralImpulse(params->f_att * calcAttraction(R2, v, id, -1));

			R2 = 1.0e10;	// infinity
			v = sm[id].body_t->getCenterOfMassPosition();
			sm[id].body_t->applyCentralImpulse(params->f_att * calcAttraction(R2, v, id, -1));

			R2 = powf(sm[id].params.hp_ratt, 2.0);
			v = sm[id].body_hp->getCenterOfMassPosition();
			sm[id].body_hp->applyCentralImpulse(params->f_att * calcAttraction(R2, v, id, 4));

			// apply repulsion forces to hold the model inside surface (f_r)
			applyRepulsion(id);
		}

		// constraint III and IV
		applyBalance(id);

		// attract hip to ground
		
		v = sm[id].body_hp->getCenterOfMassPosition();

		if (v.y() - params->h_floor > sm[id].params.hp_rbody)
		{
			btVector3 f(0.0, params->h_floor - v.y(), 0.0);
			btVector3 anc(0.0, 1.0, 0.0);
			
			sm[id].body_hp->applyImpulse(params->f_att * f * params->f_hip2floor * 100.0, anc);
		}

		sm[id].body_hd->activate(true);
		sm[id].body_n->activate(true);
		sm[id].body_t->activate(true);
		sm[id].body_hp->activate(true);
	}
}

void RodentTracker::applyBalance(int id)
{
	btTransform ts;
	btQuaternion quat;

	btVector3 posT, posN;
	btVector3 posL1, posL2;
	btVector3 posL1_t, posL2_t;
	btVector3 y;

	btVector3 V_T_N;

	float yaw, pitch;
	float d;

	
	// constraint III (feedback force to prevent rotaion of trunk along rostral-caudal axis)
	ts = sm[id].body_t->getCenterOfMassTransform();
	posL1 = ts * btVector3(0, 0, -1);
	posL2 = ts * btVector3(0, 0, 1);

	posT = sm[id].body_t->getCenterOfMassPosition();
	posN = sm[id].body_n->getCenterOfMassPosition();

	V_T_N = posN - posT;

	yaw = acos(V_T_N.x() / sqrt(V_T_N.x()*V_T_N.x() + V_T_N.z()*V_T_N.z()));
	if (V_T_N.z() < 0) yaw = -yaw;
	pitch = atan(V_T_N.y() / sqrt(V_T_N.x()*V_T_N.x() + V_T_N.z()*V_T_N.z()));

	ts.setIdentity();
	ts.setOrigin(btVector3(0, 0, 0));
	quat.setEulerZYX(-pitch, yaw, 0);
	ts.setRotation(quat);

	posL1_t = ts.getBasis() * posL1;
	posL2_t = ts.getBasis() * posL2;

	d = posL1_t.y() - posL2_t.y();

	y = ts.getBasis().inverse() * btVector3(0, 1, 0);

	sm[id].body_t->applyImpulse(params->f_const3 * (-d) * y, posL1);
	sm[id].body_t->applyImpulse(params->f_const3 * d    * y, posL2);

	// constraint IV (shift hip back when it is more front than trunk)
	
	btVector3 posHD = sm[id].body_hd->getCenterOfMassPosition();
	btVector3 posHP = sm[id].body_hp->getCenterOfMassPosition();

	float a = (posHP.x() - posT.x())*(posHD.x() - posT.x()) + (posHP.z() - posT.z())*(posHD.z() - posT.z());
	if (a>0) sm[id].body_hp->translate(btVector3(posT.x() - posHP.x(), 0, posT.z() - posHP.z()));
}

void RodentTracker::updateDistMat(void)
{
	int i, j, k;
	btVector3 bp;
	btVector3 cp;

	for (i = 0; i<dist_mat.size(); i++)
	{
		for (j = 0; j<dist_mat[i].size(); j++) dist_mat[i][j].clear();
		dist_mat[i].clear();
	}
	dist_mat.clear();

	dist_mat.resize(sm.size());
	for (i = 0; i<dist_mat.size(); i++)
	{
		dist_mat[i].resize(4);
		for (j = 0; j<4; j++)
		{
			if (j == 0) bp = sm[i].body_hd->getCenterOfMassPosition();
			else if (j == 1) bp = sm[i].body_n->getCenterOfMassPosition();
			else if (j == 2) bp = sm[i].body_t->getCenterOfMassPosition();
			else if (j == 3) bp = sm[i].body_hp->getCenterOfMassPosition();

			dist_mat[i][j].resize(pc_to_fit.size());
			for (k = 0; k<dist_mat[i][j].size(); k++)
			{
				cp.setValue(pc_to_fit[k].x, pc_to_fit[k].y, pc_to_fit[k].z);
				dist_mat[i][j][k] = bp.distance(cp);
			}
		}
	}

}

void RodentTracker::updatePointAttribution(void)
{
	bool flag_in;
	int i, j;

	p_attribution.clear();
	p_attribution.resize(pc_to_fit.size());

	for (i = 0; i<pc_to_fit.size(); i++)
	{
		p_attribution[i].animal_id = -1;
		p_attribution[i].bodypart_id = -1;

		for (j = 0; j<sm.size(); j++)
		{
			flag_in = false;

			if (dist_mat[j][0][i] < sm[j].params.hd_ratt)
			{
				if (p_attribution[i].bodypart_id < 0) p_attribution[i].bodypart_id = 1;
				else p_attribution[i].bodypart_id = 9999;
				flag_in = true;
			}

			if (dist_mat[j][1][i] < sm[j].params.n_ratt1)
			{
				if (p_attribution[i].bodypart_id < 0) p_attribution[i].bodypart_id = 1;
				else p_attribution[i].bodypart_id = 9999;
				flag_in = true;
			}
			if (dist_mat[j][2][i] < sm[j].params.t_ratt)
			{
				if (p_attribution[i].bodypart_id < 0) p_attribution[i].bodypart_id = 3;
				else p_attribution[i].bodypart_id = 9999;
				flag_in = true;
			}
			if (dist_mat[j][3][i] < sm[j].params.hp_ratt)
			{
				if (p_attribution[i].bodypart_id < 0) p_attribution[i].bodypart_id = 4;
				else p_attribution[i].bodypart_id = 9999;
				flag_in = true;
			}

			if (flag_in)
			{
				if (p_attribution[i].animal_id < 0) p_attribution[i].animal_id = j;
				else p_attribution[i].animal_id = 9999;
			}
		}
	}
}

btVector3 RodentTracker::calcAttraction(float R2, btVector3 & ori, int animal_id, int bodypart_id)
{
	btVector3 cog;
	float d2;
	int i;

	cog.setValue(0, 0, 0);

	for (i = 0; i<pc_to_fit.size(); i++)
	{
		if (bodypart_id == 1 || bodypart_id == 4)
		{
			if (p_attribution[i].animal_id == animal_id && p_attribution[i].bodypart_id == 3) continue;
			//if (tgt_part == 4 && (g_3Dimg[i].target_rat != rat_id  || g_3Dimg[i].target_part != tgt_part)) continue;
			btVector3 pos_p(pc_to_fit[i].x, pc_to_fit[i].y, pc_to_fit[i].z);
			btVector3 nrg_p(pc_to_fit[i].normal_x, pc_to_fit[i].normal_y, pc_to_fit[i].normal_z);
			btVector3 pb_vec = pos_p - sm[animal_id].body_t->getCenterOfMassPosition();
			pb_vec.normalize();

			float a = nrg_p.dot(pb_vec);

			if (a>-0.3)
			{
				btVector3 cntr(pc_to_fit[i].x, pc_to_fit[i].y, pc_to_fit[i].z);

				d2 = cntr.distance2(ori);

				if (d2<R2)
				{
					cog = cog + cntr - ori;
				}
			}
		}
		else
		{
			if (p_attribution[i].animal_id != animal_id) continue;
			if (bodypart_id > 0 && p_attribution[i].bodypart_id != bodypart_id) continue;

			btVector3 cntr(pc_to_fit[i].x, pc_to_fit[i].y, pc_to_fit[i].z);

			d2 = cntr.distance2(ori);

			if (d2<R2)
			{
				cog = cog + cntr - ori;
			}
		}
	}

	return cog;
}

void RodentTracker::applyRepulsion(int animal_id)
{
	int i, j;
	float Rrep;
	btRigidBody *rb;


	for (i = 0; i<4; i++)
	{
		if (i == 0) Rrep = sm[animal_id].params.hd_rrep;
		else if (i == 1) Rrep = sm[animal_id].params.n_rrep;
		else if (i == 2) Rrep = sm[animal_id].params.t_rrep;
		else if (i == 3) Rrep = sm[animal_id].params.hp_rrep;

		if (i == 0) rb = sm[animal_id].body_hd;
		else if (i == 1) rb = sm[animal_id].body_n;
		else if (i == 2) rb = sm[animal_id].body_t;
		else if (i == 3) rb = sm[animal_id].body_hp;

		for (j = 0; j<pc_to_fit.size(); j++)
		{
			if (dist_mat[animal_id][i][j] > Rrep) continue;

			btVector3 pos_p(pc_to_fit[j].x, pc_to_fit[j].y, pc_to_fit[j].z);
			btVector3 pb_vec = rb->getCenterOfMassPosition() - pos_p;

			if (pb_vec.x()*pc_to_fit[j].normal_x + pb_vec.y()*pc_to_fit[j].normal_y + pb_vec.z()*pc_to_fit[j].normal_z  < 0) {
				rb->applyCentralImpulse(params->f_rep * pb_vec.normalize());
			}
		}
	}
}

int RodentTracker::correctUpsideDown(void)
{
	btVector3 hd, hp, hphd, tk;
	int i, ret;

	ret = -1;

	for (i = 0; i<sm.size(); i++)
	{
		hd = sm[i].body_hd->getCenterOfMassPosition();
		hp = sm[i].body_hp->getCenterOfMassPosition();

		hphd = hd - hp;

		if (sqrt(hphd.x()*hphd.x() + hphd.z()*hphd.z()) / sqrt(hphd.x()*hphd.x() + hphd.y()*hphd.y() + hphd.z()*hphd.z()) > 0.866)
		{
			tk.setValue(0.0, 1.0, 0.0);
			tk = sm[i].body_t->getCenterOfMassTransform() * tk;
			if (tk.y() < -0.5)
			{
				float pitch, yaw;

				Eigen::Vector3f hd2(hd.x(), hd.y(), hd.z());
				Eigen::Vector3f hp2(hp.x(), hp.y(), hp.z());

				Eigen::Vector3f d = hd2 - hp2;

				yaw = acos(d.x() / sqrt(d.x()*d.x() + d.z()*d.z()));
				if (d.z() < 0) yaw = -yaw;
				pitch = atan(d.y() / sqrt(d.x()*d.x() + d.z()*d.z()));

				setModelPos(i, hp2.x() + d.x() / 2, hp2.y() + d.y() / 2, hp2.z() + d.z() / 2, pitch, -yaw);
				ret = i;
			}
		}
	}
	return ret;
}

void RodentTracker::initialize()
{
	physim = std::shared_ptr<MyPhysicsSim>(new MyPhysicsSim());

	resetSkeletonModel();

	physim->setGround(params->h_floor);

	point_force_active.clear();
	for (int i = 0; i < sm.size(); i++) point_force_active.push_back(true);
}

void RodentTracker::resetSkeletonModel()
{
	btTransform ts;
	btQuaternion quat;

	float w = 1.0;

	btTransform pos0;
	pos0.setIdentity();

	sm.clear();
	sm.resize(params->n_animal);

	calcSkeletonModelForEachAnimal();

	int id;
	for (id = 0; id < sm.size(); id++)
	{
		//T Body
		btSphereShape *shpC = new btSphereShape(sm[id].params.t_rbody);
		ts.setIdentity();
		ts.setOrigin(btVector3(0, -sm[id].params.t_rbody, 0));
		sm[id].body_t = physim->addRB(w, pos0*ts, shpC, COL_SKELTON, COL_SKELTON | COL_FLOOR);
		
		//HP Body
		btSphereShape *shpHP = new btSphereShape(sm[id].params.hp_rbody);
		ts.setIdentity(); 
		ts.setOrigin(btVector3(-sm[id].params.t_hp_len, -sm[id].params.hp_rbody, 0));
		sm[id].body_hp = physim->addRB(w, pos0*ts, shpHP, COL_SKELTON, COL_SKELTON | COL_FLOOR);

		//N Body
		btSphereShape *shpN = new btSphereShape(sm[id].params.n_rbody);
		ts.setIdentity();
		ts.setOrigin(btVector3((sm[id].params.t_n_len - sm[id].params.n_rbody), 0, 0));
		sm[id].body_n = physim->addRB(w, pos0*ts, shpN, COL_SKELTON, COL_SKELTON | COL_FLOOR);

		//HD Body
		btSphereShape *shpHD = new btSphereShape(sm[id].params.hd_rbody);
		ts.setIdentity();
		ts.setOrigin(btVector3((sm[id].params.t_n_len + sm[id].params.hd_rbody), 0, 0));
		sm[id].body_hd = physim->addRB(w, pos0*ts, shpHD, COL_SKELTON, COL_SKELTON | COL_FLOOR);

		// Joint
		btVector3 axis1;
		btVector3 axis2;
		btVector3 anchor;

		// T-N Joint
		axis1.setValue(0, 0, 1);
		axis2.setValue(0, 1, 0);
		anchor = pos0 * btVector3(0, 0, 0); 
		
		sm[id].joint_t_n = new btUniversalConstraint(*(sm[id].body_t), *(sm[id].body_n),
			anchor, pos0.getBasis()*axis1, pos0.getBasis()*axis2);
		sm[id].joint_t_n->setAngularLowerLimit(btVector3(0, -DEG2RAD(sm[id].params.t_n_angl), -DEG2RAD(sm[id].params.t_n_angd)));
		sm[id].joint_t_n->setAngularUpperLimit(btVector3(0, DEG2RAD(sm[id].params.t_n_angr), DEG2RAD(sm[id].params.t_n_angv)));
		physim->addConstraint(sm[id].joint_t_n, true);

		// N_HD Joint
		axis1.setValue(0, 0, 1);
		axis2.setValue(0, 1, 0);
		anchor = pos0 * btVector3(sm[id].params.t_n_len, 0, 0);
		
		sm[id].joint_n_hd = new btUniversalConstraint(*(sm[id].body_n), *(sm[id].body_hd),
			anchor, pos0.getBasis()*axis1, pos0.getBasis()*axis2);
		sm[id].joint_n_hd->setAngularLowerLimit(btVector3(0, -DEG2RAD(sm[id].params.n_hd_angl), -DEG2RAD(sm[id].params.n_hd_angd)));
		sm[id].joint_n_hd->setAngularUpperLimit(btVector3(0, DEG2RAD(sm[id].params.n_hd_angr), DEG2RAD(sm[id].params.n_hd_angv)));
		physim->addConstraint(sm[id].joint_n_hd, true);

		// HP_T Joint
		axis1.setValue(0, 0, 1);
		axis2.setValue(0, 1, 0); 
		anchor = pos0 * btVector3(-sm[id].params.t_hp_len, 0, 0);

		
		sm[id].joint_hp_t = new btUniversalConstraint(*(sm[id].body_hp), *(sm[id].body_t),
			anchor, pos0.getBasis()*axis1, pos0.getBasis()*axis2); 
		sm[id].joint_hp_t->setAngularLowerLimit(btVector3(0, 0, -DEG2RAD(sm[id].params.hp_t_angp)));
		sm[id].joint_hp_t->setAngularUpperLimit(btVector3(0, 0, DEG2RAD(sm[id].params.hp_t_anga)));
		physim->addConstraint(sm[id].joint_hp_t, true);

	}

	sm_pos0.hd = sm[0].body_hd->getCenterOfMassTransform();
	sm_pos0.n = sm[0].body_n->getCenterOfMassTransform();
	sm_pos0.t = sm[0].body_t->getCenterOfMassTransform();
	sm_pos0.hp = sm[0].body_hp->getCenterOfMassTransform();

	for (id = 0; id < sm.size(); id++)
	{
		setModelPos(id, 0, params->h_floor, 0.5*(float)id, 0, 0);
	}
}

void RodentTracker::calcSkeletonModelForEachAnimal()
{
	int id;
	for (id = 0; id < sm.size(); id++)
	{
		float r = 0.001 * (params->animal_size[id] / 100.0);

		sm[id].params = params->standard_model;

		sm[id].params.t_n_len *= r;
		sm[id].params.t_hp_len *= r;

		sm[id].params.hd_rbody *= r;
		sm[id].params.hd_ratt *= r;
		sm[id].params.hd_rrep *= r;

		sm[id].params.n_rbody *= r;
		sm[id].params.n_ratt1 *= r;
		sm[id].params.n_ratt2 *= r;
		sm[id].params.n_rrep *= r;

		sm[id].params.t_rbody *= r;
		sm[id].params.t_ratt *= r;
		sm[id].params.t_rrep *= r;

		sm[id].params.hp_rbody *= r;
		sm[id].params.hp_ratt *= r;
		sm[id].params.hp_rrep *= r;
	}
}

void RodentTrackerResult::save(const char * filename)
{
	int i, j;

	FILE *fp = fopen(filename, "wb");

	fwrite(&n_animal, sizeof(int), 1, fp);
	fwrite(&n_frame, sizeof(long), 1, fp);
	fwrite(&last_frame, sizeof(long), 1, fp);

	for (i = 0; i < n_animal; i++)
	{
		for (j = 0; j < n_frame; j++)
		{
			saveTransform(pos[i][j].hd, fp);
			saveTransform(pos[i][j].n, fp);
			saveTransform(pos[i][j].t, fp);
			saveTransform(pos[i][j].hp, fp);
		}
	}

	for (i = 0; i < n_animal; i++) for (j = 0; j < n_frame; j++)
	{
		unsigned char c;
		if (sm_enabled[i][j]) c = 1;
		else c = 0;
		fwrite(&c, sizeof(unsigned char), 1, fp);
	}

	fclose(fp);
}

void RodentTrackerResult::load(const char * filename)
{
	int i, j;

	FILE *fp = fopen(filename, "rb");

	fread(&n_animal, sizeof(int), 1, fp);
	fread(&n_frame, sizeof(long), 1, fp);
	fread(&last_frame, sizeof(long), 1, fp);

	for (i = 0; i < n_animal; i++)
	{
		for (j = 0; j < n_frame; j++)
		{
			loadTransform(pos[i][j].hd, fp);
			loadTransform(pos[i][j].n, fp);
			loadTransform(pos[i][j].t, fp);
			loadTransform(pos[i][j].hp, fp);
		}
	}

	for (i = 0; i < n_animal; i++) for (j = 0; j < n_frame; j++)
	{
		unsigned char c;
		fread(&c, sizeof(unsigned char), 1, fp);

		if (c == 1) sm_enabled[i][j] = true;
		else sm_enabled[i][j] = false;
	}

	fclose(fp);
}

void RodentTrackerResult::exportAsCsv(std::vector<float> & timestamp, const char * filename)
{
	int i, j;

	FILE *fp = fopen(filename, "w");

	fprintf(fp, "N of animals:, %d \n", n_animal);
	
	fprintf(fp, "frame, time, ");
	for (j = 0; j < n_animal; j++)
	{
		fprintf(fp, "Model-%d enabled, ", j+1);
	}
	for (j = 0; j < n_animal; j++)
	{
		fprintf(fp, "Head_x, Head_y, Head_z, Neck_x, Neck_y, Neck_z, Trunk_x, Trunk_y, Trunk_z, Hip_x, Hip_y, Hip_z, ");
	}
	fprintf(fp, "\n");
	
	for (i = 0; i <= last_frame; i++)
	{
		fprintf(fp, "%d, %lf, ", i, (timestamp[i]-timestamp[0])/1000.0);

		for (j = 0; j < n_animal; j++)
		{
			if (sm_enabled[j][i]) fprintf(fp, "1, ");
			else fprintf(fp, "0, ");
		}

		for (j = 0; j < n_animal; j++)
		{
			writeCenterInCsv(pos[j][i].hd, fp);
			writeCenterInCsv(pos[j][i].n, fp);
			writeCenterInCsv(pos[j][i].t, fp);
			writeCenterInCsv(pos[j][i].hp, fp);
		}

		fprintf(fp, "\n");
	}

	fclose(fp);

}

void RodentTrackerResult::initialize(int num_animal, long num_frame)
{
	int i, j;

	last_frame = -1;
	n_animal = num_animal;
	n_frame = num_frame;

	for (i = 0; i < pos.size(); i++)
	{
		pos[i].clear();
	}
	pos.clear();

	pos.resize(n_animal);
	for (i = 0; i < pos.size(); i++) pos[i].resize(n_frame);


	for (i = 0; i < sm_enabled.size(); i++)
	{
		sm_enabled[i].clear();
	}
	sm_enabled.clear();

	sm_enabled.resize(n_animal);
	for (i = 0; i < sm_enabled.size(); i++) sm_enabled[i].resize(n_frame);

	for (i = 0; i < n_animal; i++) for (j = 0; j < n_frame; j++) sm_enabled[i][j] = true;

}

void saveTransform(btTransform & transform, FILE * fp)
{
	btTransformFloatData data;

	transform.serialize(data);

	fwrite(data.m_basis.m_el[0].m_floats, sizeof(float), 4, fp);
	fwrite(data.m_basis.m_el[1].m_floats, sizeof(float), 4, fp);
	fwrite(data.m_basis.m_el[2].m_floats, sizeof(float), 4, fp);
	fwrite(data.m_origin.m_floats, sizeof(float), 4, fp);
}

void loadTransform(btTransform & transform, FILE * fp)
{
	btTransformFloatData data;

	fread(data.m_basis.m_el[0].m_floats, sizeof(float), 4, fp);
	fread(data.m_basis.m_el[1].m_floats, sizeof(float), 4, fp);
	fread(data.m_basis.m_el[2].m_floats, sizeof(float), 4, fp);
	fread(data.m_origin.m_floats, sizeof(float), 4, fp);

	transform.deSerialize(data);
}

void writeCenterInCsv(btTransform & transform, FILE * fp)
{
	fprintf(fp, "%lf, %lf, %lf, ", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
}
