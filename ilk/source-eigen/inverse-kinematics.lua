local constants = require('ilk.constants')
local keywords = require('ilk.keywords')

local generate_pose_name = function(ik)
    return ik.target..'__wrt__'..ik.reference
end

local generate_rotation_name = function(ik)
    return ik.reference..'__R__'..ik.target
end

local gen_ik_velocity_computations = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ik = args.ik
    local identifiers = args.identifiers
    local use_aux = false
    local aux_arg = ''
    local ls_first_arg = 'J'
    local temp_name = generate_pose_name(ik)
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        use_aux = true
        aux_arg = constants.ik_aux_argument.linear
        ls_first_arg = 'aux'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        use_aux = true
        aux_arg = constants.ik_aux_argument.angular
        ls_first_arg = 'aux'
    end
    local input_type = args.robot_name..'::'..constants.input_type

    local ok, res = utils.preproc([[

$(space)void $(solver_id)(const mc_config& mc, const $(input_type)& q, const $(constants.backend_namespace)::vector3_t &vector, $(input_type)& qd_ik)
$(space){
$(space)    $(constants.pose_type) $(temp_name);
$(space)    $(constants.jacobian_type) J;
$(space)    $(fk_solver)(mc, q, $(temp_name), J);
@if use_aux then
$(space)    Matrix<3,6> aux = J.block<3,6>($(constants.backend_namespace)::$(aux_arg),0);
@end
$(space)    leastSquaresSolve($(ls_first_arg), vector, qd_ik);
$(space)}

]], {
        table = table,
        pairs = pairs,
        ik = ik,
        constants = constants,
        solver_id = args.solver_id,
        fk_solver = ik.fk,
        use_aux = use_aux,
        aux_arg = aux_arg,
        ls_first_arg = ls_first_arg,
        temp_name = temp_name,
        space = utils.gen_spaces('\t', idx),
        input_type = input_type,
        identifiers = identifiers
    })
    if not ok then error(res) end
    fd:write(res)
end

local gen_ik_position_computations = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ik = args.ik
    local identifiers = args.identifiers

    local pos_temp_name = generate_pose_name(ik)
    local rot_temp_name = generate_rotation_name(ik)
    local compute_pos = false
    local compute_or = false
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        compute_pos = true
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        compute_or = true
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        compute_pos = true
        compute_or = true
    end
    local input_type = args.robot_name..'::'..constants.input_type

    local ok, res = utils.preproc([[

$(space)void $(solver_id)(const mc_config& mc, const $(constants.backend_namespace)::ik_pos_cfg& cfg,
@if compute_pos == true then
$(space)            const $(constants.backend_namespace)::vector3_t& desired_position,
@end
@if compute_or == true then
$(space)            const $(constants.backend_namespace)::rot_m_t& desired_orientation,
@end
$(space)            const $(input_type)& q_guess,
$(space)            $(input_type)& q_ik, $(constants.backend_namespace)::ik_pos_dbg &dbg)
$(space){
$(space)    using namespace std;
@if compute_pos == true then
$(space)    vector3_t ee_err_pos;
@end
@if compute_or == true then
$(space)    AxisAngle ee_err_or;
@end
$(space)
$(space)    twist_t ik_twist(twist_t::Zero());
$(space)    $(constants.jacobian_type) J;
$(space)    $(constants.pose_type) $(pos_temp_name);
$(space)    $(constants.input_type) qd;
$(space)    q_ik = q_guess;
$(space)    double ep = cfg.eps_pos_err_norm*10;
$(space)    double eo = cfg.eps_or_err_norm*10;
$(space)
$(space)    dbg.iter_count = 0;
@if compute_or == true then
$(space)    Matrix<3,3> $(rot_temp_name);
@end
$(space)
$(space)    while( (ep > cfg.eps_pos_err_norm || eo > cfg.eps_or_err_norm) && dbg.iter_count < cfg.max_iter)
$(space)    {
$(space)        $(fk_solver)(mc, q_ik, $(pos_temp_name), J);
@if compute_or == true then
$(space)        $(rot_temp_name) = eg_get_rotation($(pos_temp_name));
@end
@if compute_pos == true then
$(space)        ee_err_pos = desired_position - eg_get_position($(pos_temp_name));
@end
@if compute_or == true then
$(space)        ee_err_or  = $(constants.backend_namespace)::orientationDistance(desired_orientation, $(rot_temp_name));
@end
@if compute_pos == true then
$(space)        linearCoords( ik_twist ) = ee_err_pos / cfg.dt;
@end
@if compute_or == true then
$(space)        angularCoords(ik_twist ) = $(rot_temp_name) * (ee_err_or.axis * std::sin(ee_err_or.angle)/cfg.dt);
@end
$(space)        leastSquaresSolve(J, ik_twist, qd);
$(space)        q_ik += qd * cfg.dt;
@if compute_pos == true then
$(space)        ep = ee_err_pos.norm();
@end
@if compute_or == true then
$(space)        eo = std::abs(ee_err_or.angle);
@end
$(space)        dbg.iter_count++;
$(space)    }
$(space)    dbg.actual_pos = eg_get_position($(pos_temp_name));
$(space)    dbg.actual_or = eg_get_rotation($(pos_temp_name));
$(space)}

]], {
        table = table,
        pairs = pairs,
        ik = ik,
        constants = constants,
        solver_id = args.solver_id,
        fk_solver = ik.fk,
        use_aux = use_aux,
        aux_arg = aux_arg,
        compute_pos = compute_pos,
        compute_or = compute_or,
        pos_temp_name = pos_temp_name,
        rot_temp_name = rot_temp_name,
        ls_first_arg = ls_first_arg,
        space = utils.gen_spaces('\t', idx),
        identifiers = identifiers,
        input_type = input_type
    })
    if not ok then error(res) end
    fd:write(res)
end

local gen_ik_velocity_headers = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ik = args.ik
    local identifiers = args.identifiers
    local use_aux = false
    local aux_arg = ''
    local ls_first_arg = 'J'
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        use_aux = true
        aux_arg = constants.ik_aux_argument.linear
        ls_first_arg = 'aux'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        use_aux = true
        aux_arg = constants.ik_aux_argument.angular
        ls_first_arg = 'aux'
    end

    local input_type = args.robot_name..'::'..constants.input_type

    local ok, res = utils.preproc([[

$(space)void $(solver_id)(const mc_config& mc, const $(input_type)& q, const $(constants.backend_namespace)::vector3_t &vector, $(input_type)& qd_ik);

]], {
        table = table,
        pairs = pairs,
        ik = ik,
        constants = constants,
        solver_id = args.solver_id,
        fk_solver = ik.fk,
        use_aux = use_aux,
        aux_arg = aux_arg,
        ls_first_arg = ls_first_arg,
        space = utils.gen_spaces('\t', idx),
        identifiers = identifiers,
        input_type = input_type
    })
    if not ok then error(res) end
    fd:write(res)
end

local gen_ik_position_headers = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ik = args.ik
    local identifiers = args.identifiers

    local compute_pos = false
    local compute_or = false
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        compute_pos = true
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        compute_or = true
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        compute_pos = true
        compute_or = true
    end
    local input_type = args.robot_name..'::'..constants.input_type

    local ok, res = utils.preproc([[

$(space)void $(solver_id)(const mc_config& mc, const $(constants.backend_namespace)::ik_pos_cfg& cfg,
@if compute_pos == true then
$(space)            const $(constants.backend_namespace)::vector3_t& desired_position,
@end
@if compute_or == true then
$(space)            const $(constants.backend_namespace)::rot_m_t& desired_orientation,
@end
$(space)            const $(input_type)& q_guess,
$(space)            $(input_type)& q_ik, $(constants.backend_namespace)::ik_pos_dbg &dbg);

]], {
        table = table,
        pairs = pairs,
        ik = ik,
        constants = constants,
        solver_id = args.solver_id,
        fk_solver = ik.fk,
        use_aux = use_aux,
        aux_arg = aux_arg,
        compute_pos = compute_pos,
        compute_or = compute_or,
        ls_first_arg = ls_first_arg,
        space = utils.gen_spaces('\t', idx),
        identifiers = identifiers,
        input_type = input_type
    })
    if not ok then error(res) end
    fd:write(res)
end



local M = {}
M.gen_ik_velocity_computations = gen_ik_velocity_computations
M.gen_ik_position_computations = gen_ik_position_computations
M.gen_ik_velocity_headers = gen_ik_velocity_headers
M.gen_ik_position_headers = gen_ik_position_headers
return M
