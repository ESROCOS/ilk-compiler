local tpl     = require('ilk.template-text').template_eval
local backend = require('ilk.julia.backend-symbols')
local jlcom   = require('ilk.julia.common')
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
  return com.tplEval_failOnError("$(result) = $(arg2) * $(arg1)", ids)
end



local jacColTemplate = {
  [keys.jointType.revolute] =
[[
«bendImport».«bend.funcs.gJacCol[jtype]»(
  poi_«J.jac»,
  «bendImport».«bend.funcs.posView»(«jpose»),
  «bendImport».«bend.funcs.zAxisView»(«jpose»),
  «utils.matrixColumn(J.jac, J.col)» )
]],
  [keys.jointType.prismatic] =
[[
«bendImport».«bend.funcs.gJacCol[jtype]»(
  «bendImport».«bend.funcs.zAxisView»(«jpose»),
  «utils.matrixColumn(J.jac, J.col)» )
]]
}

M.jacobian = function(context, program)
  local bend = context.backendModule
  return {
    init = function(jac_op)
             return {
               [1] = jac_op.name .. " = " .. context.matrixInitExpr(6,program.source.meta.joint_space_size),
               [2] = "poi_"..jac_op.name.." = "..bend.."."..backend.funcs.posView.."("..program.poseValueExpression(jac_op.pose)..")"
             }
           end,
    column = function(jac_col)
               local codetpl = jacColTemplate[jac_col.jtype]
               local tplenv = {
                 bendImport = bend,
                 bend  = backend,
                 jtype = jac_col.jtype,
                 jpose = program.poseValueExpression(jac_col.joint_pose),
                 J = jac_col,
                 utils = jlcom
               }
               local ok,res = tpl(codetpl, tplenv, {xtendStyle=true, returnTable=true})
               return res
             end
  }
end


M.jointVelocity = function(context, program, jointVelOp, jointVelocitySpec, qd_argument)
  local ids = {
    qd  = qd_argument.name,
    velocity = jointVelOp.arg,
    index   = jointVelocitySpec.coordinate+1,
    spatialIndex = backend.spatialVectorIndex[ jointVelocitySpec.jtype ],
    funcs =  backend.funcs,
    init = context.matrixInitExpr(6,1)
  }

  local texttpl = nil
  if jointVelocitySpec.polarity == -1 then
    ids.H = program.poseValueExpression(jointVelocitySpec.ctransform)
    texttpl =
[[«velocity» = «init»;
 aux = «init»;
aux[«spatialIndex»] = -«qd»[«index»];
«funcs.ct_twist»( «H», aux, «velocity» );
]]
  else
    texttpl =
[[«velocity» = «init»;
«velocity»[«spatialIndex»] = «qd»[«index»];]]
  end
  return com.tplEval_failOnError(texttpl, ids, {xtendStyle=true, returnTable=true})
end





M.jointVelComposeSnippets = function(context, program, velocityComposeOp, env)
  env.coordt    = context.qualifiedBackendFunction("ct_twist")
  env.twistInit = context.matrixInitExpr(6,1)

  env.sptInd = backend.spatialVectorIndex[ env.jv.jtype ]
  env. qdInd = env.jv.coordinate

  local eval = function(text)
    local ok,res = tpl(text, env, {xtendStyle=true})
    return res
  end
  return {
    initializeVelocityVariable = eval("«res» = «twistInit»;"),
    subtractJointVelocity      = eval("«v2»[«sptInd»] -= «qd»[«qdInd»];"),
    twistCoordinateTransform   = eval("«coordt»( «H», «v2», «res» );"),
    addJointVelocity           = eval("«res»[«sptInd»] += «qd»[«qdInd»];")
  }
end



M.closuresOnContext = function (context)
  local addContextToArgs = function(...)
    local args = table.pack(...)
    local newargs = { [1] = context }
    for _,arg in ipairs(args) do
      table.insert( newargs, arg)
    end
    return table.unpack(newargs)
  end
  return {
    poseCompose = M.poseCompose,
    jacobian        = function(...) return M.jacobian(addContextToArgs(...)) end,
    jointVelocity   = function(...) return M.jointVelocity(addContextToArgs(...)) end,
    velocityCompose = function(...) return M.velocityCompose(addContextToArgs(...)) end,
    jointVelComposeSnippets = function(...) return M.jointVelComposeSnippets( addContextToArgs(...) ) end
  }
end

return M