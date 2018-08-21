local M = {}
M.backend_namespace = 'kul'
M.pose_type = 'pose_t'
M.input_type = 'joint_state'
M.jacobian_type = 'Jacobian_t'
M.const_prefix = 'mc'
M.position_type = 'vector3_t'
M.orientation_type = 'rot_m_t'
M.velocity_linear_type = 'vector3_t'
M.velocity_angular_type = 'vector3_t'
M.twist_type = 'twist_t'
M.axis_angle_type = 'AxisAngle'
local ik_aux_argument = {}
ik_aux_argument.linear = 'LX'
ik_aux_argument.angular = 'AX'
M.ik_aux_argument = ik_aux_argument


return M

