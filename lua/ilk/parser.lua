local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

-- Some of the keys of the Lua-hosted ILK models. These keys shall not be
-- visible outside this parser, which is the responsible for parsing the inputs
-- and create the internal representation of the models.
local keys = {
    robot_name = "robot_name",
    solver_type= "solver_type",
    solver_id  = "solverid",
    joints = "joints",
    jointType = {
      key = 'kind',
      prismatic = 'prismatic',
      revolute  = 'revolute'
    },
    imperativeBlock='ops',
    poses='poses',
    joint_vel_twists = 'joint_vel_twists',
    joint_acc_twists = 'joint_acc_twists',
    outputs='outputs',
    ik = {
      kind = {velocity = 'vel', position = 'pos' },
      cfgSpace = {lin='linear', ang='angular', pose='pose'},
      fkSolver = "fk"
    }
}

local M = {}

local function getJointSpaceSize(program)
  local dofs = 0
  for j, jspec in pairs(program[keys.joints]) do
    local kind = jspec[keys.jointType.key]
    if kind == keys.jointType.prismatic or kind == keys.jointType.revolute then
      dofs = dofs + 1
    else
      logger.warning("Unknown joint type '" .. kind .. "' for joint '" .. j .. "'")
    end
  end
  return dofs
end


local get_metainfo = function(program)
  local jsSize = 0
  local ilkSolverType = program[keys.solver_type]
  local solverKind = ilkSolverType
  local specs = {}
  local outputs = nil

  if ilkSolverType == "forward" then
    solverKind = M.keys.solverKind.fk
    outputs    = program[keys.outputs]
    if program[keys.joints] == nil then
      logger.warning("Could not find the 'joints' section in source program '" .. program[keys.solver_id] .. "'")
    else
      jsSize = getJointSpaceSize(program)
    end
  elseif ilkSolverType == "inverse" then
    if program.kind == keys.ik.kind.velocity then
      solverKind = M.keys.solverKind.ik.velocity
    elseif program.kind == keys.ik.kind.position then
       solverKind = M.keys.solverKind.ik.position
    end
    local space = program.vectors
    if space == keys.ik.cfgSpace.lin then
      specs.configSpace = "location"
    elseif space ==  keys.ik.cfgSpace.ang then
      specs.configSpace = "orientation"
    elseif space ==  keys.ik.cfgSpace.pose then
      specs.configSpace = "pose"
    else
      error("Unknown config-space spec: '"..space.."'. In solver "..program[keys.solver_id])
    end
    specs.fkSolverID = program[ keys.ik.fkSolver ]
    outputs = {}
    outputs["q_ik"] = {
      usersort = 1,
      otype = "jointState"
    }
  end

  return {
    robot_name = program[keys.robot_name],
    joint_space_size = jsSize,
    joints = program[keys.joints],
    solver_type= solverKind,
    solver_specs=specs,
    solver_id  = program[keys.solver_id],
    outputs    = outputs
  }
end





M.parse = function(ilk_program)
  local ret = {}
  ret.meta = get_metainfo(ilk_program)
  ret.ops = ilk_program[keys.imperativeBlock]

  ret.model_poses = ilk_program[keys.poses]

  -- Joint velocity twists
  ret.joint_vel_twists = {}
  local joint_vel_twists = ilk_program[keys.joint_vel_twists]
  if joint_vel_twists ~= nil then
    for k,v in pairs(joint_vel_twists) do
      if ret.meta.joints[ v.joint ] == nil then
        logger.warning("Velocity '" .. k .. "' references non existing joint '" .. v.joint .."'")
      end
    end
    ret.joint_vel_twists = joint_vel_twists
  end

  -- Joint acceleration twists
  ret.joint_acc_twists = {}
  local jatwists = ilk_program[keys.joint_acc_twists]
  if jatwists ~= nil then
    for k,v in pairs(jatwists) do
      if ret.meta.joints[ v.joint ] == nil then
        logger.warning("Acceleration '" .. k .. "' references non existing joint '" .. v.joint .."'")
      end
    end
    ret.joint_acc_twists = jatwists
  end
  return ret
end

-- These are some of the keys of the _parsed models_, not of the input models.
-- That is, the keys of the internal representation of the solver-model, which
-- could be different than what we take as input (and we parse).
-- These keys can be used by the code generators of this tool, which of course
-- work on the internal representation.
M.keys = {
  solverKind = {
    fk = "fk",
    ik = { position = "ikpos", velocity = "ikvel"}
  },
  ops = {
      pose_compose = 'pose-compose',
      jacobian = 'geom-jacobian',
      jacobian_column = 'GJac-col',
      joint_vel = 'joint-vel',
      joint_vel_twist = 'joint-vel-twist',
      vel_compose = 'vel-compose',
      joint_acc_twist = 'joint-acc-twist',
      acc_compose = 'acc-compose'
  },
  jointType = {
      prismatic = 'prismatic',
      revolute  = 'revolute'
  },
  ctDir = { -- coordinate transform direction
      a_x_b = 'a_x_b',
      b_x_a = 'b_x_a'
  },
  outputs = {
    itemValueKeys = {
      type = 'otype',
      sort = 'usersort'
    },
    outtypes = {
      pose = 'pose',
      velocity = 'velocity',
      jacobians = 'jacobian'
    }
  }

}


return M
