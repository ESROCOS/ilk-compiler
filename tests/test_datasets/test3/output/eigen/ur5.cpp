/*
 * Source code generated by the ILK-compiler, C++/Eigen backend
 * File generated on: Mon Nov 19 21:48:51 2018 (UTC)
 */
#include "ur5.h"
#include <ilk/eigen/gjac.h>

using namespace kul;

ur5::ModelConstants::ModelConstants()
{
    camera__fr_link2.setIdentity();
    eg_set_position(camera__fr_link2,0.1,0.2,0.3);
    eg_set_rotation(camera__fr_link2,-0.0,-1.0,0.0,
                                 1.0,-0.0,-0.0,
                                 0.0,0.0,1.0);

    fr_link1__fr_jB.setIdentity();
    eg_set_position(fr_link1__fr_jB,-1.0,0.0,-0.0);
    eg_set_rotation(fr_link1__fr_jB,1.0,-0.0,0.0,
                                 -0.0,-0.0,-1.0,
                                 0.0,1.0,-0.0);

    fr_link2__fr_jC.setIdentity();
    eg_set_position(fr_link2__fr_jC,-0.0,-0.0,-1.0);
    eg_set_rotation(fr_link2__fr_jC,1.0,0.0,0.0,
                                 -0.0,1.0,0.0,
                                 0.0,-0.0,1.0);

    fr_link3__fr_jD.setIdentity();
    eg_set_position(fr_link3__fr_jD,-1.0,-0.0,-0.0);
    eg_set_rotation(fr_link3__fr_jD,1.0,0.0,0.0,
                                 -0.0,-0.0,1.0,
                                 0.0,-1.0,-0.0);

}

void ur5::fk1(const ModelConstants& mc, const joint_state& q, kul::pose_t& fr_link1__fr_link3, kul::pose_t& fr_jA__fr_link4, kul::pose_t& fr_jA__fr_jC)
{
    pose_t fr_jA__fr_link1;
    pose_t fr_jB__fr_link2;
    pose_t fr_jC__fr_link3;
    pose_t fr_jD__fr_link4;

    rot_z__b_x_a(q(0), fr_jA__fr_link1);
    tr_z__b_x_a(q(1), fr_jB__fr_link2);
    rot_z__b_x_a(q(2), fr_jC__fr_link3);
    tr_z__b_x_a(q(3), fr_jD__fr_link4);

    pose_t fr_jB__fr_link2 = fr_jB__fr_link2 * mc.fr_link1__fr_jB;
    pose_t fr_link2__fr_link3 = fr_jC__fr_link3 * mc.fr_link2__fr_jC;
    pose_t fr_link3__fr_link4 = fr_jD__fr_link4 * mc.fr_link3__fr_jD;
    fr_link1__fr_link3 = fr_link2__fr_link3 * fr_link1__fr_link2;
    pose_t fr_link1__fr_link4 = fr_link3__fr_link4 * fr_link1__fr_link3;
    fr_jA__fr_link4 = fr_link1__fr_link4 * fr_jA__fr_link1;
    pose_t fr_link1__fr_jC = mc.fr_link2__fr_jC * fr_link1__fr_link2;
    fr_jA__fr_jC = fr_link1__fr_jC * fr_jA__fr_link1;


}



