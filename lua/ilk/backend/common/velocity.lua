local utils       = require('ilk.common')
local langcommons = require("ilk.backend.common.common")


local function velocityCompose (program, velocityComposeOp, qd_argument, lang)
  local jv = program.source.joint_vel_twists[ velocityComposeOp.arg1 ]
         -- nil if 'arg1' is not a joint velocity
  if jv == nil then
    langcommons.logger.warning("Velocity composition only supports joint-induced velocity, at the moment")
    return "//WARNING: Velocity composition only supports joint-induced velocity, at the moment"
  end
  local ids = {
    qd  = qd_argument.name,
    res = velocityComposeOp.res,
    v1  = velocityComposeOp.arg1,
    v2  = velocityComposeOp.arg2,
    H   = velocityComposeOp.pose,
    joint = program.source.meta.joints[ jv.joint ],
    jv  = jv
  }
  local env = {
    lang = lang.jointVelComposeSnippets(program, velocityComposeOp, ids),
    jv   = jv
  }
  local texttpl =
[[
${lang.initializeVelocityVariable}
@ if jv ~= nil then
  @ if jv.polarity == -1 then
${lang.subtractJointVelocity}
${lang.twistCoordinateTransform}
    @ else
${lang.twistCoordinateTransform}
${lang.addJointVelocity}
    @ end
  @ else
#### velocity composition with no joint velocity argument --- TODO
@ end
]]
  local code = utils.tplEval_failOnError(texttpl, env, {xtendStyle=true, returnTable=true})
  return code
end


local function jointVelocityTwist(program, op, qd_argument, lang)
  if qd_argument == nil then
    langcommons.logger.error("The joint velocity metaargument is nil, cannot proceed")
    langcommons.fail()
  end

  local jointTwistSpec = program.source.joint_vel_twists[op.arg]
  if jointTwistSpec == nil then
    langcommons.logger.error("Specification for the joint velocity " .. op.arg .. " not found in program " .. program.source.meta.solver_id)
    langcommons.fail()
  end

  local joint = program.source.meta.joints[ jointTwistSpec.joint ]
  if joint == nil then
    langcommons.logger.error("Joint twist '" .. op.arg .. "' references non existing joint '" .. jointTwistSpec.joint .."'")
    langcommons.fail()
  end

  local code = lang.jointVelocityTwist(program, op, jointTwistSpec, joint, qd_argument)
  return code
end

return {
  velocityCompose    = velocityCompose,
  jointVelocityTwist = jointVelocityTwist
}
