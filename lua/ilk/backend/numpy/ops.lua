local tpl     = require('template-text').template_eval
local backend = require('ilk.backend.numpy.backend-symbols')
local pycom   = require('ilk.backend.numpy.common')
local com     = require('ilk.common')
local keys    = require('ilk.parser').keys

local M = {}


M.poseCompose = function(program, poseComposeOp)
  local ids = {
      result = poseComposeOp.res,
      arg1   = program.poseValueExpression(poseComposeOp.arg1),
      arg2   = program.poseValueExpression(poseComposeOp.arg2)
  }
  return com.tplEval_failOnError("$(result) = $(arg2) @ $(arg1)", ids, {returnTable=true})
end



local jacColTemplate = {
  [keys.jointType.revolute] =
[[
«bendImport».«bend.funcs.gJacCol[jtype]»(
  poi_«J.jac»,
  «bendImport».«bend.funcs.posView»(«jpose»),
  «bendImport».«bend.funcs.zAxisView»(«jpose»),
  «bend.matrixColumn(J.jac, J.col)» )
]],
  [keys.jointType.prismatic] =
[[
«bendImport».«bend.funcs.gJacCol[jtype]»(
  «bendImport».«bend.funcs.zAxisView»(«jpose»),
  bend.matrixColumn(J.jac, J.col)» )
]]
}


M.geometricJacobianInit = function(program, op, config)
  local bend = config.importBackendAs
  return {
    [1] = op.name .. " = " ..bend.. "." ..backend.matrixT(6, program.source.meta.joint_space_size),
    [2] = "poi_"..op.name.." = "..bend.."."..backend.funcs.posView.."("..program.poseValueExpression(op.pose)..")"
  }
end

M.geometricJacobianColumn = function(program, op, joint, config)
  local codetpl = jacColTemplate[joint.kind]
  local tplenv = {
     bendImport = config.importBackendAs,
     bend  = backend,
     jtype = joint.kind,
     jpose = program.poseValueExpression(op.joint_pose),
     J = op
   }
   local ok,res = tpl(codetpl, tplenv, {xtendStyle=true, returnTable=true})
   return res
end


M.jointVelocityTwist = function(program, jointVelOp, jointVelocitySpec, joint, qd_argument, config)
  local ids = {
    qd  = qd_argument.name,
    velocity = jointVelOp.arg,
    index   = joint.coordinate,
    spatialIndex = backend.spatialVectorIndex[ joint.kind ],
    coordt = config.importBackendAs .. "." .. backend.funcs.ct_twist,
    init = backend.matrixT(6,1),
  }

  local texttpl = nil
  if jointVelocitySpec.polarity == -1 then
    ids.H = program.poseValueExpression(jointVelocitySpec.ctransform)
    texttpl =
[[«velocity» = «init»
aux = «init»
aux[«spatialIndex»] = -«qd»[«index»]
«coordt»( «H», aux, «velocity» )
]]
  else
    texttpl =
[[«velocity» = «init»
«velocity»[«spatialIndex»] = «qd»[«index»] ]]
  end
  return com.tplEval_failOnError(texttpl, ids, {xtendStyle=true, returnTable=true})
end




M.jointVelComposeSnippets = function(program, velocityComposeOp, env, config)
  env.coordt    = config.importBackendAs .. "." .. backend.funcs.ct_twist
  env.twistInit = backend.matrixT(6,1)

  env.sptInd = backend.spatialVectorIndex[ env.joint.kind ]
  env. qdInd = env.joint.coordinate

  local eval = function(text)
    local ok,res = tpl(text, env, {xtendStyle=true,returnTable=true})
    return res
  end
  return {
    initializeVelocityVariable = eval("«res» = «twistInit»"),
    subtractJointVelocity      = eval("«v2»[«sptInd»] -= «qd»[«qdInd»]"),
    twistCoordinateTransform   = eval("«coordt»( «H», «v2», «res» )"),
    addJointVelocity           = eval("«res»[«sptInd»] += «qd»[«qdInd»]")
  }
end


M.closuresOnConfig = function (config)
local addConfigToArgs = function(...)
  local args = table.pack(...)
  table.insert(args, config)
  return table.unpack(args)
end
return {
  poseCompose = M.poseCompose,
  geometricJacobianInit  = function(...) return M.geometricJacobianInit(addConfigToArgs(...)) end,
  geometricJacobianColumn= function(...) return M.geometricJacobianColumn(addConfigToArgs(...)) end,
  jointVelocityTwist     = function(...) return M.jointVelocityTwist(addConfigToArgs(...)) end,
  velocityCompose = function(...) return M.velocityCompose(addConfigToArgs(...)) end,
  jointVelComposeSnippets = function(...) return M.jointVelComposeSnippets( addConfigToArgs(...) ) end
}
end

return M
