-- Some of the keys of the Lua-hosted ILK models. These keys shall not be
-- visible outside this parser, which is the responsible for parsing the inputs
-- and create the internal representation of the models.
local keys = {
    robot_name = "robot_name",
    joint_space = "joint_space_size",
    solver_type= "solver_type",
    solver_id  = "solverid",
    imperativeBlock='ops',
    poses='poses',
    joint_velocities = 'joint_velocities',
    outputs='outputs',
    ik = {
      kind = {velocity = 'vel', position = 'pos' },
      cfgSpace = {lin='linear', ang='angular', pose='pose'},
      fkSolver = "fk"
    }
}

local M = {}


local get_metainfo = function(program)
  local ilkSolverType = program[keys.solver_type]
  local solverKind = ilkSolverType
  local specs = {}
  local outputs = nil

  if ilkSolverType == "forward" then
    solverKind = M.keys.solverKind.fk
    outputs    = program[keys.outputs]
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
    joint_space_size = program[keys.joint_space] or 6,
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

  local ilkposes = ilk_program[keys.poses]
  if ilkposes ~= nil then
    ret.model_poses = { constant=ilkposes.constant, joint={} }
    for k,v in pairs(ilkposes.joint) do
      ret.model_poses.joint[k] = { jtype=v.jtype, dir=v.dir, coordinate=v.input}
    end
  end

  ret.joint_velocities = {}
  local jVelocities = ilk_program[keys.joint_velocities]
  if jVelocities ~= nil then
    for k,v in pairs(jVelocities) do
      ret.joint_velocities[k] = { jtype=v.jtype, coordinate=v.index, polarity=v.polarity, ctransform=v.ctransform }
    end
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
      jacobian = 'geom-jacobian',
      jacobian_column = 'GJac-col',
      joint_vel = 'joint-vel',
      vel_joint_explicit = 'vel-joint',
      vel_compose = 'vel-compose',
      pose_compose = 'pose-compose'
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
