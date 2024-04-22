local tpl     = require('template-text').template_eval
local backend = require('ilk.backend.eigen.backend-symbols')
local cppcom  = require('ilk.backend.eigen.common')
local com     = require('ilk.common')
local keys    = require('ilk.parser').keys

local M = {}

local tpleval = function(template, env, opts)
  return com.tplEval_failOnError(template, env, opts or {xtendStyle=true, returnTable=true})
end


M.poseCompose = function(program, poseComposeOp)
  local ids = {
      result = poseComposeOp.res,
      arg1   = program.poseValueExpression(poseComposeOp.arg1),
      arg2   = program.poseValueExpression(poseComposeOp.arg2)
  }
  if program.source.meta.outputs[poseComposeOp.res]==nil then
    ids.result = backend.types[com.metatypes.pose] .. " " .. ids.result
  end
  return com.tplEval_failOnError("$(result) = $(arg2) * $(arg1);", ids, {returnTable=true})
end




local jacColTemplate = {
  [keys.jointType.revolute] =
[[
geometricJacobianColumn_revolute(
  poi_«J.jac»,
  «funcs.posView»(«jpose»),
  «polarity»«funcs.zAxisView»(«jpose»),
  «J.jac».col(«J.col») );
]],
  [keys.jointType.prismatic] =
[[
geometricJacobianColumn_prismatic(
  «polarity»«funcs.zAxisView»(«jpose»),
  «J.jac».col(«J.col») );
]]
}

M.jacobian = function(program)
  return {
    init = function(jac_op)
             return {
               [1] = backend.types[com.metatypes.position3d].." poi_"..jac_op.name.." = "..backend.funcs.posView.."("..jac_op.pose..");"
             }
           end,
    column = function(jac_col)
               local codetpl = jacColTemplate[jac_col.jtype]
               local polarity = ""
               if jac_col.polarity == -1 then
                 polarity = "-"
               end
               local ok,res = tpl(codetpl,
                 {
                   jpose = program.poseValueExpression(jac_col.joint_pose),
                   funcs = backend.funcs,
                   J = jac_col,
                   polarity = polarity
                 }, {xtendStyle=true, returnTable=true})
               return res
             end
  }
end

M.geometricJacobianInit = function(program, op)
  return {
    [1] = backend.types[com.metatypes.position3d].." poi_"..op.name.." = "..backend.funcs.posView.."("..op.pose..");"
  }
end

M.geometricJacobianColumn = function(program, op, joint)
  local codetpl = jacColTemplate[joint.kind]
  local polarity = ""
  if op.polarity == -1 then
    polarity = "-"
  end
  local code = com.tplEval_failOnError(codetpl,
     {
       jpose = program.poseValueExpression(op.joint_pose),
       funcs = backend.funcs,
       J = op,
       polarity = polarity
     }, {xtendStyle=true, returnTable=true})
  return code
end


M.jointVelocityTwist = function(program, jointVelOp, jointVelocitySpec, joint, qd_argument)
  local twist_t = backend.types[com.metatypes.twist]
  local ids = {
    qd  = qd_argument.name,
    velocity = jointVelOp.arg,
    index   = joint.coordinate,
    spatialIndex = backend.spatialVectorIndex[ joint.kind ],
    funcs =  backend.funcs
  }
  if program.source.meta.outputs[jointVelOp.arg]==nil then
    ids.init = twist_t .. " " .. jointVelOp.arg .. " = " .. twist_t .. "::Zero();"
  else
    ids.init = jointVelOp.arg .. ".setZero();"
  end

  local texttpl = nil
  if jointVelocitySpec.polarity == -1 then
    ids.twist_t = twist_t
    ids.H = program.poseValueExpression(jointVelocitySpec.ctransform)
    texttpl =
[[«init»
«twist_t» aux = twist_t::Zero();
aux[«spatialIndex»] = -«qd»(«index»);
«funcs.ct_twist»( «H», aux, «velocity» );
]]
  else
    texttpl =
[[«init»
«velocity»[«spatialIndex»] = «qd»(«index»);]]
  end
  return com.tplEval_failOnError(texttpl, ids, {xtendStyle=true, returnTable=true})
end



M.jointVelComposeSnippets = function(program, velocityComposeOp, env)
  local defineLocalForResult = program.source.meta.outputs[velocityComposeOp.res]==nil
        -- the resulting velocity is not an output of the solver, so there is no
        -- formal parameter for it; thus, (generate code to) define a local variable.

  local twistInit = ""
  if defineLocalForResult then
    env.vel_t = backend.types[com.metatypes.velocity]
    twistInit = "«vel_t» «res»;"
  end
  env.coordt = backend.funcs.ct_twist
  env.sptInd = backend.spatialVectorIndex[ env.joint.kind ]
  env. qdInd = env.joint.coordinate

  local eval = function(text) return tpleval(text, env) end
  return {
    initializeVelocityVariable = eval(twistInit),
    subtractJointVelocity      = eval("«v2»[«sptInd»] -= «qd»[«qdInd»];"),
    twistCoordinateTransform   = eval("«coordt»( «H», «v2», «res» );"),
    addJointVelocity           = eval("«res»[«sptInd»] += «qd»[«qdInd»];")
  }
end

M.jointAccelerationTwist = function(program, op, jointAccTwistSpec, joint, qdd_arg)
  local twist_t = backend.types[com.metatypes.twist]
  local ids = {
    qdd = qdd_arg.name,
    acc = op.arg,
    index = joint.coordinate,
    spatialIndex = backend.spatialVectorIndex[ joint.kind ],
    funcs =  backend.funcs
  }
  if program.source.meta.outputs[op.arg]==nil then
    ids.init = twist_t .. " " .. op.arg .. " = " .. twist_t .. "::Zero();"
  else
    ids.init = op.arg .. ".setZero();"
  end

  local texttpl = nil
  if op.polarity == -1 then
    cppcom.logger.warning("Joint acceleration twist with inverse polarity not supported yet")
    texttpl =
[[«init»
#error Inverse polarity acceleration not supported yet
]]
  else
    texttpl =
[[«init»
«acc»[«spatialIndex»] = «qdd»(«index»);]]
  end
  return com.tplEval_failOnError(texttpl, ids, {xtendStyle=true, returnTable=true})
end


M.forwardPropagationAcceleration = function(program, op, env)
  local defineLocalForResult = program.source.meta.outputs[op.res]==nil
  local twistInit = ""
  if defineLocalForResult then
    env.acc_t = backend.types[com.metatypes.acceleration]
    twistInit = "«acc_t» «res»;"
  end
  env.coordt = backend.funcs.ct_twist
  env.sptInd = backend.spatialVectorIndex[ env.joint.kind ]
  env.Sdot   = backend.funcs.Sdot[ env.joint.kind ]
  env.vel    = op.velocity

  local lines = function(text) return tpleval(text, env, {xtendStyle=true, returnTable=true}) end
  return {
    initializeAccelerationVariable = lines(twistInit),
    subtractJointAcceleration      = lines("«a2»[«sptInd»] -= «qdd»[«joint.coordinate»];"),
    twistCoordinateTransform       = lines("«coordt»( «H», «a2», «res» );"),
    addRelativeAcceleration = lines(
[[«res» += «Sdot»(«vel») * «qd»[«joint.coordinate»];
«res»[«sptInd»] += «qdd»[«joint.coordinate»];
]]
)
  }
end


return M
