/*
  Source code library generated with ILK-GEN
  File generated on: Thu May 31 22:00:03 2018 (UTC)
*/
#include "ur5.h"
#include <ilk/eigen/gjac.h>

using namespace kul;
using namespace ur5;

mc_config::mc_config() {
camera__fr_link2.setIdentity();
eg_set_position(camera__fr_link2,0.1,0.2,0.3);
eg_set_rotation(camera__fr_link2,-0,-1,0,
                     1,-0,-0,
                     0,0,1);

fr_link1__fr_jB.setIdentity();
eg_set_position(fr_link1__fr_jB,-1,0,-0);
eg_set_rotation(fr_link1__fr_jB,1,-0,0,
                     -0,-0,-1,
                     0,1,-0);

fr_link2__fr_jC.setIdentity();
eg_set_position(fr_link2__fr_jC,-0,-0,-1);
eg_set_rotation(fr_link2__fr_jC,1,0,0,
                     -0,1,0,
                     0,-0,1);

fr_link3__fr_jD.setIdentity();
eg_set_position(fr_link3__fr_jD,-1,-0,-0);
eg_set_rotation(fr_link3__fr_jD,1,0,0,
                     -0,-0,1,
                     0,-1,-0);

}

void fk1(const mc_config& mc, const ur5::joint_state& input, kul::pose_t& fr_link1__fr_link3, kul::pose_t& fr_jA__fr_link4, kul::pose_t& fr_jA__fr_jC) {
	pose_t fr_jB__fr_link2;
	pose_t fr_jC__fr_link3;
	pose_t fr_jA__fr_link1;
	pose_t fr_jD__fr_link4;
	tr_z__b_x_a(input(1),fr_jB__fr_link2);
	rot_z__b_x_a(input(2),fr_jC__fr_link3);
	rot_z__b_x_a(input(0),fr_jA__fr_link1);
	tr_z__b_x_a(input(3),fr_jD__fr_link4);


	pose_t fr_link1__fr_link2 = fr_jB__fr_link2 * mc.fr_link1__fr_jB;
	pose_t fr_link2__fr_link3 = fr_jC__fr_link3 * mc.fr_link2__fr_jC;
	pose_t fr_link3__fr_link4 = fr_jD__fr_link4 * mc.fr_link3__fr_jD;
	fr_link1__fr_link3 = fr_link2__fr_link3 * fr_link1__fr_link2;
	pose_t fr_link1__fr_link4 = fr_link3__fr_link4 * fr_link1__fr_link3;
	fr_jA__fr_link4 = fr_link1__fr_link4 * fr_jA__fr_link1;
	pose_t fr_link1__fr_jC = mc.fr_link2__fr_jC * fr_link1__fr_link2;
	fr_jA__fr_jC = fr_link1__fr_jC * fr_jA__fr_link1;

}
    
