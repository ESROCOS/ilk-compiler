local langcommons = require("ilk.backend.common.common")



local geometricJacobian = function(program, op, lang)
  if not op.columns then
    langcommons.logger.error("Geometric Jacobian opcode lacks the 'columns' field (for Jacobian '" .. op.name .. "')")
    langcommons.fail()
  end
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
