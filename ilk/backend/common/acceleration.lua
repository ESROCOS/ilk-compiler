local utils       = require('ilk.common')
local langcommons = require("ilk.backend.common.common")



local accelerationCompose = function(program, op, qd_arg, qdd_arg, lang)
  local ja = program.source.joint_acc_twists[ op.arg1 ]
         -- nil if 'arg1' is not a joint-induced acceleration
  if ja == nil then
    langcommons.logger.warning("Acceleration composition only supports joint-induced acceleration, at the moment")
    return "//WARNING: Acceleration composition only supports joint-induced acceleration, at the moment"
  end
  local ids = {
    qd  = qd_arg.name,
    qdd = qdd_arg.name,
    res = op.res,
    a1  = op.arg1,
    a2  = op.arg2,
    H   = op.ctransform,
    joint = program.source.meta.joints[ ja.joint ],
    ja  = ja
  }
  local env = {
    lang = lang.forwardPropagationAcceleration(program, op, ids),
    ja   = ja
  }
  local texttpl =
[[
${lang.initializeAccelerationVariable}
@ if ja ~= nil then
  @ if ja.polarity == -1 then
${lang.subtractJointAcceleration}
${lang.twistCoordinateTransform}
    @ else
${lang.twistCoordinateTransform}
${lang.addRelativeAcceleration}
    @ end
  @ else
#### acceleration composition with no joint-induced acceleration argument --- TODO
@ end
]]
  return utils.tplEval_failOnError(texttpl, env, {xtendStyle=true, returnTable=true})
end


local jointAccelerationTwist = function(program, op, qdd_argument, lang)
  if qdd_argument == nil then
    langcommons.logger.error("The joint acceleration metaargument is nil, cannot proceed")
    langcommons.fail()
  end

  local jointTwistSpec = program.source.joint_acc_twists[op.arg]
  if jointTwistSpec == nil then
    langcommons.logger.error("Specification for the joint acceleration twist " .. op.arg .. " not found in program " .. program.source.meta.solver_id)
    langcommons.fail()
  end

  local joint = program.source.meta.joints[ jointTwistSpec.joint ]
  if joint == nil then
    langcommons.logger.error("Joint twist '" .. op.arg .. "' references non existing joint '" .. jointTwistSpec.joint .."'")
    langcommons.fail()
  end

  local code = lang.jointAccelerationTwist(program, op, jointTwistSpec, joint, qdd_argument)
  return code
end



return {
  accelerationCompose    = accelerationCompose,
  jointAccelerationTwist = jointAccelerationTwist
}
