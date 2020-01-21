local langcommons = require("ilk.backend.common.common")



local geometricJacobian = function(program, op, lang)
  return lang.geometricJacobianInit(program, op)
end

local geometricJacobianColumn = function(program, op, lang)
  local joint = program.source.meta.joints[ op.joint ]
  if joint == nil then
    langcommons.logger.error("Jacobian column for '" .. op.jac .. "' references non existing joint '" .. op.joint .."'")
    langcommons.fail()
  end
  return lang.geometricJacobianColumn(program, op, joint)
end


return {
  geometricJacobian = geometricJacobian,
  geometricJacobianColumn = geometricJacobianColumn
}