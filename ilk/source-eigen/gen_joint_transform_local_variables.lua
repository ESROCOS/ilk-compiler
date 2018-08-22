local constants = require('ilk.constants')

local is_already_declared_as_output = function(v, outputs)
    for j, w in pairs(outputs) do
        if (v.name == w.target) then
            return true
        end
    end
    return false
end

local find_identifiers_that_need_declaration = function(args, outputs)
    local need_declaration = {}
    for i, v in pairs(args) do
        if not is_already_declared_as_output(v, outputs) then
            need_declaration[#need_declaration + 1] = v
        end
    end
    return need_declaration
end

local gen_joint_transform_local_variables = function(config, args, outputs, idx)
    -- args is list of op 'model_T_joint_local'
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local pose_type = constants.pose_type
    local to_generate = find_identifiers_that_need_declaration(args, outputs)

    local ok, res = utils.preproc([[
@for i,v in pairs(vars) do
$(space)$(pose_type) $(v.name);
@end
]], {
        table = table,
        pairs = pairs,
        pose_type = pose_type,
        space = utils.gen_spaces('\t', idx),
        vars = to_generate
    })
    if not ok then error(res) end
    fd:write(res)
end


return gen_joint_transform_local_variables