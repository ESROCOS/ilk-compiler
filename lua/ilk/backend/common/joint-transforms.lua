local utils       = require('ilk.common')
local langcommons = require("ilk.backend.common.common")


local setJointTransforms = function(program, backend)
  local arg_q = langcommons.inputArgumentByType(program, utils.metatypes.jointState)

  local code = {}
  for name,info in utils.alphabPairs(program.source.model_poses.joint) do
    local joint = program.source.meta.joints[ info.joint ]
    if joint == nil then
      langcommons.logger.warning("Could not find joint '"..info.joint.."' in program '"..program.source.meta.solver_id.."'")
    end
    local line  = backend.jointTransformSetvalue(program,  {direction=info.dir, transformName=name, joint=joint, statusVectVarName=arg_q.name} )
    table.insert(code, line)
  end
  return code
end

return {
  setJointTransforms = setJointTransforms
}