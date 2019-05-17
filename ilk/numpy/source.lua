--[[
   Module for generating source code of an ilk program in Python/Numpy
   License: BSD 2-clause
--]]

local tpl = require('ilk.template-text').template_eval
local com = require('ilk.common')
local keys = require("ilk.parser").keys
local langcom = require('ilk.langcommons')

local backend  = require('ilk.numpy.backend-symbols')
local numpycom = require('ilk.numpy.common')
local numpyops = require('ilk.numpy.ops')


local M = {}




local modelConstsCTor = function(context, config)

  local env = {
    struct = "ModelConstants",
    modelValues = context.outer.modelValues,
    sorted = com.alphabPairs,

    setOneCode = function(poseid, value)
      local ok,res = tpl([[
«poseid» = «backend».pose()
«backend».«funcs.setPosition»(«poseid»,«val.p[1]»,«val.p[2]»,«val.p[3]»)
«backend».«funcs.setRotation»(«poseid»,«val.r[1]»,«val.r[2]»,«val.r[3]»,
                             «val.r[4]»,«val.r[5]»,«val.r[6]»,
                             «val.r[7]»,«val.r[8]»,«val.r[9]»)
]],   {funcs = backend.funcs, backend = config.importBackendAs, poseid=poseid, val=value},
      {xtendStyle=true, returnTable=true})
      return res
    end
  }

  local ret = {}
  local ok,res = tpl([[
def «struct»():
    @for k,_ in sorted(modelValues) do
    @  local one = setOneCode(k, modelValues[k] )
    ${one}

    @end

    mc_config = namedtuple('mc', [
    @for k,_ in sorted(modelValues) do
    '«k»',
    @end
    ])

    mc = mc_config(
    @for k,_ in sorted(modelValues) do
    «k» = «k»,
    @end
    )
    return mc


]], env,
  {xtendStyle=true, returnTable=true})
  return res
end


M.heading = function(context, config)
  local env = {
    cfg = config,
    date = os.date("!%c"),
    mcCTor = modelConstsCTor(context, config)
  }
  local ok, res = tpl(
[[
# This file was automatically generated by the ILK-compiler on «date»

import math
import numpy as np
import numpy.linalg as nplin
from collections import namedtuple

from ilknumpy import backend as «cfg.importBackendAs»

${mcCTor}
]], env, {verbose=true, xtendStyle=true})
  return res
end


M.fksource = function(program, context, config)
  local numpy_specifics = numpyops.closuresOnConfig(config)
  numpy_specifics.jointTransformSetvalue = backend.jointTransformSetvalue(config.importBackendAs)

  local env = {
      bend = backend,
      signature = program.signature.toString({declaration=false}),
      body = langcom.code.motionSweep(program, context, numpy_specifics),
      returns = numpycom.returnStatement(program)
  }
  local templ =
[[
«signature»

    ${body}

    «returns»

]]
  return com.tplEval_failOnError(templ, env,
                    {verbose=true, xtendStyle=true, returnTable = false})
end


local ikgen = {

[keys.solverKind.ik.velocity] =
function(program, context, config)
  local configSpace = program.source.meta.solver_specs.configSpace
  local env = {
    metat    = com.metatypes,
    typeh    = numpycom,
    backend = config.importBackendAs,
    signature= program.signature,
    fkSolver = program.fkSolver,
    fkargs = {
      [1] = program.signature.inputs[1].name,
      [2] = program.signature.inputs[2].name
    },
    leastSquares = backend.funcs.lsSolve,
    configSpace  = configSpace,
    jacVar = "J"
  }

  local templ =
[[
«signature.toString({declaration=false})»
    _, «jacVar» = «fkSolver.signature.toString({call=true, args=fkargs})»
@if configSpace ~= "pose" then
    «typeh.matrixBlockExpr(jacVar, 6, configSpace ).defAndInit("aux")»
@   jacVar = "aux"
@end
    «signature.outputs[1].name» = «backend».«leastSquares»(«jacVar», «signature.inputs[3].name»)
    return «signature.outputs[1].name»
]]
  return com.tplEval_failOnError(templ, env, {verbose=true, xtendStyle=true})
end,


[keys.solverKind.ik.position] =
function(program, context, config)
  local configSpace = program.source.meta.solver_specs.configSpace
  local cfgVar = program.signature.inputs[2].name
  local env = {
    metat    = com.metatypes,
    signature= program.signature,
    fkSolver = program.fkSolver,
    fkargs = {
      [1] = program.signature.inputs[1].name,
      [2] = program.signature.outputs[1].name -- the joint status
    },
    cfgVar = cfgVar,
    backend = backend,
    funcs= backend.funcs,
    backendImport = config.importBackendAs
  }
  env.deso = nil
  env.desp = nil
  env.compute_pos = false
  env.compute_or = false
  env.while_conditions = ''
  env.returns = numpycom.returnStatement(program)
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
      env.while_conditions = '((ep > '..cfgVar..'.eps_pos_err_norm) or (eo > '..cfgVar..'.eps_or_err_norm))'
      env.desp = program.signature.inputs[3].name
      env.deso = program.signature.inputs[4].name
  end
  local templ=
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

    «dbg» = «backendImport».«backend.ik_pos_dbg»()
@if compute_or then
    R = «backend.matrixT(3,3)»
@end
    ik_twist = «backend.matrixT(6,1)»
    angview  = «backendImport».«funcs.angularCoords»( ik_twist )
    linview  = «backendImport».«funcs.linearCoords»( ik_twist )
    while( «while_conditions» and «dbg».iter_count < «cfgVar».max_iter):
        pose, Jacobian = «fkSolver.signature.toString({call=true, args=fkargs})»
@if compute_or then
        R = «backendImport».«funcs.rotView»(pose)
        ee_err_or  = «backendImport».«funcs.orientDist»(«deso», R)
        angview[:] = R @ (ee_err_or.axis * math.sin(ee_err_or.angle)/«cfgVar».dt)
@end
@if compute_pos then
        ee_err_pos = «desp» - «backendImport».«funcs.posView»(pose)
        linview[:] = ee_err_pos / «cfgVar».dt
@end
        qd = «backendImport».«funcs.lsSolve»(Jacobian, ik_twist)
        «q_ik» += qd * «cfgVar».dt
@if compute_pos then
        ep = np.linalg.norm(ee_err_pos)
@end
@if compute_or then
        eo = abs(ee_err_or.angle)
@end
        «dbg».iter_count = «dbg».iter_count + 1

    «dbg».actual_pos = «backendImport».«funcs.posView»(pose)
    «dbg».actual_or  = «backendImport».«funcs.rotView»(pose)
    «returns»
]]
  return com.tplEval_failOnError(templ, env, {verbose=true, xtendStyle=true})
end

}



M.iksource = function(program, context, config)
  if program.source.meta.solver_type ~= keys.solverKind.ik.position and
     program.source.meta.solver_type ~= keys.solverKind.ik.velocity then
    error("This generator is meant only for inverse kinematics routines")
  end
  return ikgen[program.source.meta.solver_type](program, context, config)
end


return M
