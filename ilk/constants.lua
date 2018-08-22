local M = {}
M.backend_namespace = 'kul'
M.pose_type = 'pose_t'
M.input_type = 'joint_state'
M.jacobian_type = 'Jacobian_t'
M.const_prefix = 'mc'
local ik_aux_argument = {}
ik_aux_argument.linear = 'LX'
ik_aux_argument.angular = 'AX'
M.ik_aux_argument = ik_aux_argument


return M

