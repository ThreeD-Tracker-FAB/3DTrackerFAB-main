#pragma once

#include <vector>


#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <iostream>
#include <fstream>

#include "my_physics_sim.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

class RodentTrackerParam {
public:

	float sim_dt;				// delta T for physics simulation
	float f_att;				// constant for f_a (attraction)
	float f_rep;				// constant for f_r (repulsion)
	float v_term;				// threshold of velocity for terminate fitting (steady state)
	float f_const3;				// force for constraint III, i.e., ensuring that the back is always toward the ceiling
	float f_hip2floor;			// attraction force to hip from floor (relative to f_att)
	float h_floor;				// floor height

	struct SkeletonModelParam
	{
		// skeleton model parameters
		// hd, n, t, hp: head, neck, trunk, hip
		// len: length between given body parts
		// rbody: radius of physical body
		// ratt: radius for attraction force (r_hd, etc. in paper)
		// rrep: radius for repulsive force (s in paper)
		// anga, angp, angl, angr, angd, angv: limitation of joint movement (anterior, posterior, left, right, dorsal, ventral)

		float t_n_len;
		float t_hp_len;

		float hd_rbody;
		float hd_ratt;
		float hd_rrep;

		float n_rbody;
		float n_ratt1;
		float n_ratt2;
		float n_rrep;

		float t_rbody;
		float t_ratt;
		float t_rrep;

		float hp_rbody;
		float hp_ratt;
		float hp_rrep;

		float n_hd_angl;
		float n_hd_angr;
		float n_hd_angd;
		float n_hd_angv;

		float t_n_angl;
		float t_n_angr;
		float t_n_angd;
		float t_n_angv;

		float hp_t_anga;
		float hp_t_angp;
	};
	
	SkeletonModelParam standard_model;

	int n_animal;						// number of animals
	std::vector<float> animal_size;		// size of each animal, ratio relative to the standard model

	int cf_hsv_min[3];		//color filter - min values
	int cf_hsv_max[3];		//color filter - max values
	bool cf_enable;			//color filter - enable or disable
	int cf_inc;				//color filter - include or exclude

	bool orf_enable;		//outlier removal filter - enable or disable
	int orf_meanK;			//outlier removal filter - meanK
	float orf_thresh;		//outlier removal filter - thresh

	RodentTrackerParam();

	void save(const char* filename);
	int load(const char* filename);
	void setNumAnimal(int n);

private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		if (version >= 1)
		{
			ar & BOOST_SERIALIZATION_NVP(sim_dt);
			ar & BOOST_SERIALIZATION_NVP(f_att);
			ar & BOOST_SERIALIZATION_NVP(f_rep);
			ar & BOOST_SERIALIZATION_NVP(v_term);
			ar & BOOST_SERIALIZATION_NVP(f_const3);
			ar & BOOST_SERIALIZATION_NVP(f_hip2floor);
			ar & BOOST_SERIALIZATION_NVP(h_floor);

			ar & BOOST_SERIALIZATION_NVP(standard_model.t_n_len);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_hp_len);

			ar & BOOST_SERIALIZATION_NVP(standard_model.hd_rbody);
			ar & BOOST_SERIALIZATION_NVP(standard_model.hd_ratt);
			ar & BOOST_SERIALIZATION_NVP(standard_model.hd_rrep);

			ar & BOOST_SERIALIZATION_NVP(standard_model.n_rbody);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_ratt1);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_ratt2);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_rrep);

			ar & BOOST_SERIALIZATION_NVP(standard_model.t_rbody);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_ratt);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_rrep);

			ar & BOOST_SERIALIZATION_NVP(standard_model.hp_rbody);
			ar & BOOST_SERIALIZATION_NVP(standard_model.hp_ratt);
			ar & BOOST_SERIALIZATION_NVP(standard_model.hp_rrep);

			ar & BOOST_SERIALIZATION_NVP(standard_model.n_hd_angl);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_hd_angr);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_hd_angd);
			ar & BOOST_SERIALIZATION_NVP(standard_model.n_hd_angv);

			ar & BOOST_SERIALIZATION_NVP(standard_model.t_n_angl);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_n_angr);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_n_angd);
			ar & BOOST_SERIALIZATION_NVP(standard_model.t_n_angv);

			ar & BOOST_SERIALIZATION_NVP(standard_model.hp_t_anga);
			ar & BOOST_SERIALIZATION_NVP(standard_model.hp_t_angp);

			ar & BOOST_SERIALIZATION_NVP(n_animal);

			ar & BOOST_SERIALIZATION_NVP(animal_size);
		}
		if (version >= 2)
		{
			ar & BOOST_SERIALIZATION_NVP(cf_hsv_min);
			ar & BOOST_SERIALIZATION_NVP(cf_hsv_max);
			ar & BOOST_SERIALIZATION_NVP(cf_enable);
			ar & BOOST_SERIALIZATION_NVP(cf_inc);
		}
		if (version >= 3)
		{
			ar & BOOST_SERIALIZATION_NVP(orf_enable);
			ar & BOOST_SERIALIZATION_NVP(orf_meanK);
			ar & BOOST_SERIALIZATION_NVP(orf_thresh);
		}
	}

};

BOOST_CLASS_VERSION(RodentTrackerParam, 3);

class RodentTracker {
public:

	struct SkeletonModel
	{
		btRigidBody *body_hp;
		btRigidBody *body_t;
		btRigidBody *body_n;
		btRigidBody *body_hd;

		btUniversalConstraint *joint_hp_t;  
		btUniversalConstraint *joint_t_n;  
		btUniversalConstraint *joint_n_hd;	

		RodentTrackerParam::SkeletonModelParam params;
	};

	struct SkeletonModelPos
	{
		btTransform hp;
		btTransform t;
		btTransform n;
		btTransform hd;
	};

	std::vector<SkeletonModel> sm;

	SkeletonModelPos sm_pos0;

	RodentTracker(std::shared_ptr<RodentTrackerParam> p);

	void setModelPos(int id, float x, float y, float z, float pitch, float yaw);
	void setModelPos(int id, SkeletonModelPos & sm_pos);
	void getModelPos(int id, SkeletonModelPos & sm_pos);

	void runSimStep();
	void runSimUntilSteadyState();

	void setPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> & pc);

	void togglePointForce(int animal_id, bool state);
	
private:

	struct PointAttribution
	{
		int animal_id;
		int bodypart_id;
	};

	std::shared_ptr<RodentTrackerParam> params;
	std::shared_ptr<MyPhysicsSim> physim;
	std::vector< std::vector< std::vector< float > > > dist_mat;	// distances between each point and each body part of each skeleton model

	pcl::PointCloud<pcl::PointXYZRGBNormal> pc_to_fit;
	std::vector<PointAttribution> p_attribution;

	std::vector<bool> point_force_active;

	void initialize();
	void resetSkeletonModel();
	void calcSkeletonModelForEachAnimal();

	void applyForce();
	void applyBalance(int id);
	void updateDistMat(void);
	void updatePointAttribution(void);

	btVector3 calcAttraction(float R2, btVector3 &ori, int animal_id, int bodypart_id);
	void applyRepulsion(int animal_id);

	int correctUpsideDown(void);
};

class RodentTrackerResult {
public:
	std::vector< std::vector<RodentTracker::SkeletonModelPos> > pos;
	std::vector< std::vector<bool>> sm_enabled;

	long last_frame;
	
	long n_frame;
	int n_animal;

	void save(const char* filename);
	void load(const char* filename);

	void exportAsCsv(std::vector<float> & timestamp, const char * filename);

	void initialize(int num_animal, long num_frame);


private:

};

// convenience functions
void saveTransform(btTransform & transform, FILE * fp);
void loadTransform(btTransform & transform, FILE * fp);
void writeCenterInCsv(btTransform & transform, FILE * fp);