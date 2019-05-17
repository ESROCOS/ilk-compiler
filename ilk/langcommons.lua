local com = require('ilk.common')

local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)
local M = { code={} }

local fail = function()
  error("In the backend-independent functions; check the log for details")
end


local velocityCompose = function(program, velocityComposeOp, qd_argument, lang)
  local jv = program.source.joint_velocities[ velocityComposeOp.arg1 ]
         -- nil if 'arg1' is not a joint velocity
  local ids = {
    qd  = qd_argument.name,
    res = velocityComposeOp.res,
    v1  = velocityComposeOp.arg1,
    v2  = velocityComposeOp.arg2,
    H   = velocityComposeOp.pose,
    jv  = jv
  }
  local env = {
    lang = lang.jointVelComposeSnippets(program, velocityComposeOp, ids),
    jv   = jv
  }
  local texttpl =
[[
«lang.initializeVelocityVariable»
@ if jv ~= nil then
  @ if jv.polarity == -1 then
«lang.subtractJointVelocity»
«lang.twistCoordinateTransform»
    @ else
«lang.twistCoordinateTransform»
«lang.addJointVelocity»
    @ end
  @ else
#### velocity composition with no joint velocity argument --- TODO
@ end
]]
  return com.tplEval_failOnError(texttpl, env, {xtendStyle=true, returnTable=true})
end




M.code.setJointTransforms = function(program, lang)
  local code = {}
  local jState = "UNKNOWN"
  for i,p in ipairs(program.signature.inputs) do
    if p.meta.metatype==com.metatypes.jointState then
      jState = p.name
    end
  end
  for name,info in com.alphabPairs(program.source.model_poses.joint) do
    local line = lang.jointTransformSetvalue( {ilkspec=info, name=name, statusVectVarName=jState} )
    table.insert(code, line)
  end
  return code
end


local tpl = require('ilk.template-text').template_eval
local keys = require("ilk.parser").keys

M.code.poseComposes = function(prog, lang)
  local code = {}
  for i,op in ipairs(prog.source.ops) do
    if op.op == keys.ops.pose_compose then
      local res = lang.poseCompose(prog, op)
      table.insert(code, res)
    end
  end
  return code
end

M.code.jointVelocities = function(prog, lang, qd_argument)
  local code = {}
  local ops, count = com.operationsByKey(prog.source, keys.ops.vel_joint_explicit)
  if count > 0 then
    if qd_argument == nil then
      logger.error("The joint velocity metaargument is nil, cannot proceed")
      fail()
    end
    for i,op in ipairs(ops) do
      local jointVelocitySpec = prog.source.joint_velocities[op.arg]
      if jointVelocitySpec == nil then
        logger.error("Specification for the joint velocity " .. op.arg .. " not found in program " .. prog.solver_id)
        fail()
      end
      local res = lang.jointVelocity(prog, op, jointVelocitySpec, qd_argument)
      for i,line in ipairs(res) do
        table.insert(code, line)
      end
    end
  end
  return code
end


M.code.velocityComposes = function(prog, lang, qd_argument)
  local code = {}
  local ops, count = com.operationsByKey(prog.source, keys.ops.vel_compose)
  if count > 0 then
    if qd_argument == nil then
      logger.error("The joint velocity metaargument is nil, cannot proceed")
      fail()
    end
    for i,op in ipairs(ops) do
      local res = velocityCompose(prog, op, qd_argument, lang)
      table.insert(code, res)
    end
  end
  return code
end


M.code.geometricJacobians = function(program, lang)
  local ret = { init={}, cols={}}
  local jacgen = lang.jacobian(program)
  for i,v in ipairs(program.source.ops) do
    if(v.op==keys.ops.jacobian) then
      table.insert(ret.init, jacgen.init(v) )
    elseif(v.op==keys.ops.jacobian_column) then
      table.insert(ret.cols, jacgen.column(v))
    end
  end
  return ret
end


M.code.motionSweep = function(program, context, languageSpecifics)
  if program.source.meta.solver_type ~= keys.solverKind.fk then
    error("This generator is meant only for forward kinematics routines")
    --TODO make this check more relaxed, for sweeping solvers
  end

  local arg_qd = M.inputArgumentByType(program, com.metatypes.jointVel)

  local env = {
      jTransf = M.code.setJointTransforms(program, languageSpecifics),
      comps   = M.code.poseComposes(program, languageSpecifics),
      jvels   = M.code.jointVelocities(program, languageSpecifics, arg_qd),
      velComp = M.code.velocityComposes(program, languageSpecifics, arg_qd),
      jacs    = M.code.geometricJacobians(program, languageSpecifics)
  }
  local template=
[[
${jTransf}

${comps}

${jvels}

  @for i,v in ipairs(velComp) do
${v}
  @end

  @for k,v in ipairs(jacs.init) do
${v}
  @end

  @for k,v in ipairs(jacs.cols) do
${v}
  @end

]]
  return com.tplEval_failOnError(template,env,{verbose=true, xtendStyle=true, returnTable=true})
end


M.metaToConcrete = function(concreteContainer, metaElement)
  for k,v in pairs(concreteContainer) do
    if k == "meta" then
      if v == metaElement then
        return concreteContainer
      end
    else
      if type(v) == "table" then
        local res = M.metaToConcrete(v, metaElement)
        if res ~= nil then
          return res
        end
      end
    end
  end
  return nil
end

M.argumentFromMeta = function(program, metaArgument)
  for i,v in ipairs(program.signature.inputs) do
    if v.meta == metaArgument then return v end
  end
  for i,v in ipairs(program.signature.outputs) do
    if v.meta == metaArgument then return v end
  end
  return nil
end

M.inputArgumentByType = function(program, type)
  local i,meta_arg = program.source.metasignature.inputByType( type )
  if meta_arg ~= nil then
    return M.argumentFromMeta(program, meta_arg) -- could be nil
  end
  return nil
end

M.outputArgumentByType = function(program, type)
  local i,meta_arg = program.source.metasignature.outputByType( type )
  if meta_arg ~= nil then
    return M.argumentFromMeta(program, meta_arg) -- could be nil
  end
  return nil
end




return M
