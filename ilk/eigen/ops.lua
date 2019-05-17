local tpl     = require('ilk.template-text').template_eval
local backend = require('ilk.eigen.backend-symbols')
local cppcom  = require('ilk.eigen.common')
local com     = require('ilk.common')
local langcom = require('ilk.langcommons')
local keys    = require('ilk.parser').keys

local M = {}


M.poseCompose = function(program, poseComposeOp)
  local ids = {
      result = poseComposeOp.res,
      arg1   = program.poseValueExpression(poseComposeOp.arg1),
      arg2   = program.poseValueExpression(poseComposeOp.arg2)
  }
  if program.source.meta.outputs[poseComposeOp.res]==nil then
    ids.result = backend.types[com.metatypes.pose] .. " " .. ids.result
  end
  local ok,res = tpl("$(result) = $(arg2) * $(arg1);", ids)
  if not ok then
    cppcom.logger.error("In the evaluation of the text template for a pose compose")
  end
  return res
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


M.jointVelocity = function(program, jointVelOp, jointVelocitySpec, qd_argument)
  local twist_t = backend.types[com.metatypes.twist]
  local ids = {
    qd  = qd_argument.name,
    velocity = jointVelOp.arg,
    index   = jointVelocitySpec.coordinate,
    spatialIndex = backend.spatialVectorIndex[ jointVelocitySpec.jtype ],
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

  env.sptInd = backend.spatialVectorIndex[ env.jv.jtype ]
  env. qdInd = env.jv.coordinate

  local eval = function(text)
    local ok,res = tpl(text, env, {xtendStyle=true})
    return res
  end
  return {
    initializeVelocityVariable = eval(twistInit),
    subtractJointVelocity      = eval("«v2»[«sptInd»] -= «qd»[«qdInd»];"),
    twistCoordinateTransform   = eval("«coordt»( «H», «v2», «res» );"),
    addJointVelocity           = eval("«res»[«sptInd»] += «qd»[«qdInd»];")
  }
end

return M