/*
  Source code library generated with ILK-GEN
  File generated on: Thu Jul 19 03:45:52 2018 (UTC)
*/
#include "ur5.h"
#include <ilk/eigen/gjac.h>

using namespace kul;
using namespace ur5;

void ik4(const mc_config& mc, const kul::ik_pos_cfg& cfg,
            const kul::vector3_t& desired_position,
            const joint_state& q_guess,
            joint_state& q_ik, kul::ik_pos_dbg &dbg)
{
    using namespace std;
    vector3_t ee_err_pos;

    twist_t ik_twist(twist_t::Zero());
    Jacobian_t J;
    pose_t temp;
    joint_state qd;
    q_ik = q_guess;
    double ep = cfg.eps_pos_err_norm*10;
    double eo = cfg.eps_or_err_norm*10;

    dbg.iter_count = 0;

    while( (ep > cfg.eps_pos_err_norm || eo > cfg.eps_or_err_norm) && dbg.iter_count < cfg.max_iter)
    {
        fk__ik1(mc, q_ik, temp, J);
        ee_err_pos = desired_position - eg_get_position(temp);
        linearCoords( ik_twist ) = ee_err_pos / cfg.dt;
        leastSquaresSolve(J, ik_twist, qd);
        q_ik += qd * cfg.dt;
        ep = ee_err_pos.norm();
        dbg.iter_count++;
    }
    dbg.actual_pos = eg_get_position(temp);
    dbg.actual_or = eg_get_rotation(temp);
}


