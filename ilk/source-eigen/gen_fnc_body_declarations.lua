local ilk_keywords = require('ilk.keywords')

local rot_or_tr = function(v)
    if v[ilk_keywords.op_argument.joint_local.joint_type] == ilk_keywords.joint_type.revolute then
        return "rot_z"
    elseif v[ilk_keywords.op_argument.joint_local.joint_type] == ilk_keywords.joint_type.prismatic then
        return "tr_z"
    end
end

local direction = function(v)
    if v[ilk_keywords.op_argument.joint_local.direction] == ilk_keywords.direction.a_x_b then
        return "a_x_b"
    elseif v[ilk_keywords.op_argument.joint_local.direction] == ilk_keywords.direction.b_x_a then
        return "b_x_a"
    end
end

local arguments = function(v)
    local input_index = v[ilk_keywords.op_argument.joint_local.input]
    local position = v[ilk_keywords.op_argument.joint_local.name]
    return input_index, position
end

local gen_fnc_body_declarations = function(config, args, idx)
    -- args is list of op 'model_T_joint_local'
    local fd = config.fd or io.stdout
    local idx = idx or 0

    local ok, res = utils.preproc([[
@for i,v in pairs(vars) do
@   local rot_or_tr = rot_or_tr(v)
@   local direction = direction(v)
@   local input_index, position = arguments(v)
$(space)$(rot_or_tr)__$(direction)(input($(input_index)),$(position));
@end


]], {
        table = table,
        pairs = pairs,
        space = utils.gen_spaces('\t', idx),
        vars = args,
        ilk_keywords = ilk_keywords,
        direction = direction,
        rot_or_tr = rot_or_tr,
        arguments = arguments
    })

    if not ok then error(res) end
    fd:write(res)
end


return gen_fnc_body_declarations

