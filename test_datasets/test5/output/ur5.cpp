/*
  Source code library generated with ILK-GEN
  File generated on: Thu Aug 23 19:54:56 2018 (UTC)
*/
#include "ur5.h"
#include <ilk/eigen/gjac.h>

using namespace kul;

ur5::mc_config::mc_config() {
fr_elbow__fr_upper_arm.setIdentity();
eg_set_position(fr_elbow__fr_upper_arm,0.425,0,-0.1197);
eg_set_rotation(fr_elbow__fr_upper_arm,1,-0,0,
                     0,1,-0,
                     0,0,1);

fr_shoulder_lift__fr_shoulder.setIdentity();
eg_set_position(fr_shoulder_lift__fr_shoulder,0.13585,0,0);
eg_set_rotation(fr_shoulder_lift__fr_shoulder,0,-0,1,
                     -0,-1,0,
                     1,-0,-0);

fr_shoulder_pan__fr_base.setIdentity();
eg_set_position(fr_shoulder_pan__fr_base,0,0,0.089159);
eg_set_rotation(fr_shoulder_pan__fr_base,1,-0,0,
                     0,1,-0,
                     0,0,1);

fr_wr1__fr_forearm.setIdentity();
eg_set_position(fr_wr1__fr_forearm,0.39225,0,0.09315);
eg_set_rotation(fr_wr1__fr_forearm,1,-0,0,
                     0,1,-0,
                     0,0,1);

fr_wr2__fr_wrist_1.setIdentity();
eg_set_position(fr_wr2__fr_wrist_1,0.09475,0,0);
eg_set_rotation(fr_wr2__fr_wrist_1,0,-0,1,
                     -0,-1,0,
                     1,-0,-0);

fr_wr3__fr_wrist_2.setIdentity();
eg_set_position(fr_wr3__fr_wrist_2,0.0825,0,0);
eg_set_rotation(fr_wr3__fr_wrist_2,-0,0,1,
                     0,1,0,
                     -1,0,-0);

}

void ur5::fk1(const ur5::mc_config& mc, const ur5::joint_state& input, kul::pose_t& fr_wrist_3__fr_base, ur5::t_J_fr_wrist_3_fr_base& J_fr_wrist_3_fr_base, ur5::t_J_fr_wrist_3_fr_elbow& J_fr_wrist_3_fr_elbow) {
	pose_t fr_wrist_3__fr_wr3;
	pose_t fr_wrist_2__fr_wr2;
	pose_t fr_wrist_1__fr_wr1;
	pose_t fr_forearm__fr_elbow;
	pose_t fr_upper_arm__fr_shoulder_lift;
	pose_t fr_shoulder__fr_shoulder_pan;
	rot_z__a_x_b(input(5),fr_wrist_3__fr_wr3);
	rot_z__a_x_b(input(4),fr_wrist_2__fr_wr2);
	rot_z__a_x_b(input(3),fr_wrist_1__fr_wr1);
	rot_z__a_x_b(input(2),fr_forearm__fr_elbow);
	rot_z__a_x_b(input(1),fr_upper_arm__fr_shoulder_lift);
	rot_z__a_x_b(input(0),fr_shoulder__fr_shoulder_pan);


	pose_t fr_wr1__fr_elbow = fr_forearm__fr_elbow * mc.fr_wr1__fr_forearm;
	pose_t fr_wr2__fr_wr1 = fr_wrist_1__fr_wr1 * mc.fr_wr2__fr_wrist_1;
	pose_t fr_wr2__fr_elbow = fr_wr1__fr_elbow * fr_wr2__fr_wr1;
	pose_t fr_wr3__fr_wr2 = fr_wrist_2__fr_wr2 * mc.fr_wr3__fr_wrist_2;
	pose_t fr_wr3__fr_elbow = fr_wr2__fr_elbow * fr_wr3__fr_wr2;
	pose_t fr_wrist_3__fr_elbow = fr_wr3__fr_elbow * fr_wrist_3__fr_wr3;
	pose_t fr_shoulder_lift__fr_shoulder_pan = fr_shoulder__fr_shoulder_pan * mc.fr_shoulder_lift__fr_shoulder;
	pose_t fr_shoulder_lift__fr_base = mc.fr_shoulder_pan__fr_base * fr_shoulder_lift__fr_shoulder_pan;
	pose_t fr_elbow__fr_shoulder_lift = fr_upper_arm__fr_shoulder_lift * mc.fr_elbow__fr_upper_arm;
	pose_t fr_elbow__fr_base = fr_shoulder_lift__fr_base * fr_elbow__fr_shoulder_lift;
	pose_t fr_wr1__fr_base = fr_elbow__fr_base * fr_wr1__fr_elbow;
	pose_t fr_wr2__fr_base = fr_wr1__fr_base * fr_wr2__fr_wr1;
	pose_t fr_wr3__fr_base = fr_wr2__fr_base * fr_wr3__fr_wr2;
	fr_wrist_3__fr_base = fr_wr3__fr_base * fr_wrist_3__fr_wr3;

	position_t poi_J_fr_wrist_3_fr_base = eg_get_position(fr_wrist_3__fr_base);
	position_t poi_J_fr_wrist_3_fr_elbow = eg_get_position(fr_wrist_3__fr_elbow);
	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(mc.fr_shoulder_pan__fr_base),
		eg_get_zaxis(mc.fr_shoulder_pan__fr_base),
		J_fr_wrist_3_fr_base.col(0));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(fr_shoulder_lift__fr_base),
		eg_get_zaxis(fr_shoulder_lift__fr_base),
		J_fr_wrist_3_fr_base.col(1));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(fr_elbow__fr_base),
		eg_get_zaxis(fr_elbow__fr_base),
		J_fr_wrist_3_fr_base.col(2));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(fr_wr1__fr_base),
		eg_get_zaxis(fr_wr1__fr_base),
		J_fr_wrist_3_fr_base.col(3));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(fr_wr2__fr_base),
		eg_get_zaxis(fr_wr2__fr_base),
		J_fr_wrist_3_fr_base.col(4));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_base,
		eg_get_position(fr_wr3__fr_base),
		eg_get_zaxis(fr_wr3__fr_base),
		J_fr_wrist_3_fr_base.col(5));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_elbow,
		eg_get_position(fr_wr1__fr_elbow),
		eg_get_zaxis(fr_wr1__fr_elbow),
		J_fr_wrist_3_fr_elbow.col(0));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_elbow,
		eg_get_position(fr_wr2__fr_elbow),
		eg_get_zaxis(fr_wr2__fr_elbow),
		J_fr_wrist_3_fr_elbow.col(1));

	geometricJacobianColumn_revolute(
		poi_J_fr_wrist_3_fr_elbow,
		eg_get_position(fr_wr3__fr_elbow),
		eg_get_zaxis(fr_wr3__fr_elbow),
		J_fr_wrist_3_fr_elbow.col(2));

}
    
