local tpl     = require('ilk.template-text').template_eval
local backend = require('ilk.numpy.backend-symbols')
local pycom   = require('ilk.numpy.common')
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
  return com.tplEval_failOnError("$(result) = $(arg2) @ $(arg1)", ids)
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

M.jacobian = function(program, config)
  local bend = config.importBackendAs

  return {
    init = function(jac_op)
             return {
               [1] = jac_op.name .. " = " ..bend.. "." ..backend.matrixT(6,6), -- TODO!! the second 6 is hardcoded!! It should be the joint space size
               [2] = "poi_"..jac_op.name.." = "..bend.."."..backend.funcs.posView.."("..program.poseValueExpression(jac_op.pose)..")"
             }
           end,
    column = function(jac_col)
               local codetpl = jacColTemplate[jac_col.jtype]
               local tplenv = {
                 bendImport = config.importBackendAs,
                 bend  = backend,
                 jtype = jac_col.jtype,
                 jpose = program.poseValueExpression(jac_col.joint_pose),
                 J = jac_col
               }
               local ok,res = tpl(codetpl, tplenv, {xtendStyle=true, returnTable=true})
               return res
             end
  }
end


M.jointVelocity = function(program, jointVelOp, jointVelocitySpec, qd_argument, config)
  local ids = {
    qd  = qd_argument.name,
    velocity = jointVelOp.arg,
    index   = jointVelocitySpec.coordinate,
    spatialIndex = backend.spatialVectorIndex[ jointVelocitySpec.jtype ],
    coordt = config.importBackendAs .. "." .. backend.funcs.ct_twist,
    init = backend.matrixT(6,1),
    bendImport = config.importBackendAs,
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

  env.sptInd = backend.spatialVectorIndex[ env.jv.jtype ]
  env. qdInd = env.jv.coordinate

  local eval = function(text)
    local ok,res = tpl(text, env, {xtendStyle=true})
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
  jacobian        = function(...) return M.jacobian(addConfigToArgs(...)) end,
  jointVelocity   = function(...) return M.jointVelocity(addConfigToArgs(...)) end,
  velocityCompose = function(...) return M.velocityCompose(addConfigToArgs(...)) end,
  jointVelComposeSnippets = function(...) return M.jointVelComposeSnippets( addConfigToArgs(...) ) end
}
end

return M
