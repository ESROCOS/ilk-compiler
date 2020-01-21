--[[
   Module for generating source code of an ilk program in Julia
   License: BSD 2-clause
--]]

local com = require('ilk.common')
local keys = require("ilk.parser").keys

local backend  = require('ilk.backend.julia.backend-symbols')
local jlcom = require('ilk.backend.julia.common')
local jlops = require('ilk.backend.julia.ops')


local M = {}

local tpl_opts = {verbose=true, xtendStyle=true, returnTable=true}
local tpl = function(template, env)
  return com.tplEval_failOnError(template, env, tpl_opts )
end


M.modelConstsCTor = function(context, config)

  local setOneCode = function(poseid, value)
      local template=
[[
«poseid» = «backend».pose()
«backend».«funcs.setPosition»(«poseid»,«val.p[1]»,«val.p[2]»,«val.p[3]»)
«backend».«funcs.setRotation»(«poseid»,«val.r[1]»,«val.r[2]»,«val.r[3]»,«val.r[4]»,«val.r[5]»,«val.r[6]»,«val.r[7]»,«val.r[8]»,«val.r[9]»)
]]
      local env = {funcs = backend.funcs, backend = context.backendModule, poseid=poseid, val=value}
      return tpl(template, env)
  end

  local env = {
    backend     = context.backendModule,
    setOneCode  = setOneCode,
    poses       = function() return com.alphabPairs(context.outer.modelValues.poses) end
  }

  local template =
[[
struct ConstPosesContainer
    @for k,_ in poses() do
    «k»::«backend».MatrixType
    @end
end

function ConstPosesInit()
    @for k, v in poses() do
    @  local one = setOneCode(k, v )
    ${one}
    @end

    ret = ConstPosesContainer(
        @for k, _ in poses() do
        «k»,
        @end
    )
    return ret
end

const ModelConstants = ConstPosesInit()

]]
  return tpl(template, env)
end


local ops_handlers    = require("ilk.backend.common.ops-handlers")
local jointTransforms = require("ilk.backend.common.joint-transforms")

M.fksource = function(program, context, opsHandlers, config)
  local aux = {
    jointTransformSetvalue = backend.jointTransformSetvalue(context.backendModule)
  }
  
  local env = {
      signature = program.signature.toString({declaration=false}),
      jTransf   = jointTransforms.setJointTransforms(program, aux),
      compiled_ops = function() return ops_handlers.translate(program, opsHandlers) end,
      returns = jlcom.returnStatement(program)
  }
  local template =
[[
«signature»

    ${jTransf}

  @ for op, code in compiled_ops() do
    ${code}
  @ end

    «returns»
end

]]
  return tpl(template, env)
end


local ikgen = {

[keys.solverKind.ik.velocity] =
function(program, context, config)
  local configSpace = program.source.meta.solver_specs.configSpace
  local env = {
    common  = jlcom,
    backend = context.backendModule,
    signature= program.signature,
    fkSolver = program.fkSolver,
    fkargs = {
      [1] = program.signature.inputs[1].name,
      [2] = program.signature.inputs[2].name
    },
    leastSquares = backend.funcs.lsSolve,
    configSpace  = configSpace,
    jacVar = "J",
    returnS = jlcom.returnStatement(program)
  }

  local template=
[[
«signature.toString({declaration=false})»
    _, «jacVar» = «fkSolver.signature.toString({call=true, args=fkargs})»
@if configSpace ~= "pose" then
    «common.matrixBlockExpr(jacVar, 6, configSpace ).defAndInit("aux")»
@   jacVar = "aux"
@end
    «signature.outputs[1].name» = «backend».«leastSquares»(«jacVar», «signature.inputs[3].name»)
    «returnS»
end
]]
  return tpl(template, env)
end,

[keys.solverKind.ik.position] =
function(program, context, config)
  local configSpace = program.source.meta.solver_specs.configSpace
  local cfgVar = program.signature.inputs[2].name
  local env = {
    matrix   = function(r,c) return context.matrixInitExpr(r,c) end,
    signature= program.signature,
    fkSolver = program.fkSolver,
    fkargs = {
      [1] = program.signature.inputs[1].name,
      [2] = program.signature.outputs[1].name -- the joint status
    },
    cfgVar = cfgVar,
    backend = backend,
    funcs= backend.funcs,
    backendModule = context.backendModule
  }
  env.deso = nil
  env.desp = nil
  env.compute_pos = false
  env.compute_or = false
  env.while_conditions = ''
  env.returns = jlcom.returnStatement(program)
  if configSpace == "location" then
      env.compute_pos = true
      env.while_conditions = '(ep > '..cfgVar..'.eps_pos_err_norm)'
      env.desp = program.signature.inputs[3].name
  elseif configSpace == "orientation" then
      env.compute_or = true
      env.while_conditions = '(eo > '..cfgVar..'.eps_or_err_norm)'
      env.deso = program.signature.inputs[3].name
  elseif configSpace == "pose" then
      env.compute_pos = true
      env.compute_or = true
      env.while_conditions = '((ep > '..cfgVar..'.eps_pos_err_norm) || (eo > '..cfgVar..'.eps_or_err_norm))'
      env.desp = program.signature.inputs[3].name
      env.deso = program.signature.inputs[4].name
  end
  local template=
[[
«signature.toString({declaration=false})»
@if compute_pos then
    ep = «cfgVar».eps_pos_err_norm*10
@end
@if compute_or then
    eo = «cfgVar».eps_or_err_norm*10
@end
@local q_ik = fkargs[2]
@local dbg  = signature.outputs[2].name

    «q_ik» = q_guess

    «dbg» = «backendModule».«backend.ik_pos_dbg»(0, «matrix(3,1)», «matrix(3,3)»)
@if compute_or then
    R = «matrix(3,3)»
@end
    ik_twist = «matrix(6,1)»
    pose = «matrix(4,4)»
    while( «while_conditions» && «dbg».iter_count < «cfgVar».max_iter)
        pose, Jacobian = «fkSolver.signature.toString({call=true, args=fkargs})»
@if compute_or then
        R = «backendModule».«funcs.rotView»(pose)
        ee_err_or  = «backendModule».«funcs.orientDist»(«deso», R)
        angview = «backendModule».«funcs.angularCoords»( ik_twist )
        angview[:] = R * (ee_err_or.axis * sin(ee_err_or.angle)/«cfgVar».dt)
@end
@if compute_pos then
        ee_err_pos = «desp» - «backendModule».«funcs.posView»(pose)
        linview = «backendModule».«funcs.linearCoords»( ik_twist )
        linview[:] = ee_err_pos / «cfgVar».dt
@end
        qd = «backendModule».«funcs.lsSolve»(Jacobian, ik_twist)
        «q_ik» += qd * «cfgVar».dt
@if compute_pos then
        ep = LinearAlgebra.norm(ee_err_pos)
@end
@if compute_or then
        eo = abs(ee_err_or.angle)
@end
        «dbg».iter_count = «dbg».iter_count + 1
    end

    «dbg».actual_pos[:] = «backendModule».«funcs.posView»(pose)
    «dbg».actual_or[:]  = «backendModule».«funcs.rotView»(pose)
    «returns»
end
]]
  return tpl(template, env)
end

}



M.iksource = function(program, context, config)
  if program.source.meta.solver_type ~= keys.solverKind.ik.position and
     program.source.meta.solver_type ~= keys.solverKind.ik.velocity then
    error("This generator is meant only for inverse kinematics routines")
  end
  local fkid = program.source.meta.solver_specs.fkSolverID
  local fkSolver, i = com.findProgramByID(fkid, context.outer)
  if fkSolver == nil then
    error("Could not find the parsed model of FK solver "..fkid..
          ", required by IK solver "..program.source.meta.solver_id)
  end
  local candidate = context.programs[i]
  if candidate.source ~= fkSolver then
    jlcom.logger.warning("Possible error in the FK solver of IK solver"..program.source.meta.solver_id)
  end
  program.fkSolver = candidate

  return ikgen[program.source.meta.solver_type](program, context, config)


end


return M
