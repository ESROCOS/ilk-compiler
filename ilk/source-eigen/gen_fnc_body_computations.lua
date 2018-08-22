local common = require('ilk.common')

local type_declaration_if_variable_was_not_yet_defined = function(id, olist)
    local result_type = 'pose_t '
    for i, v in pairs(olist) do
        if id == v.target then
            return ""
        end
    end
    return result_type
end



-- This handles the generation of the concrete computation
local gen_fnc_body_computations = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local compose = args.compose
    local model_const = args.model_const
    local olist = args.outputs
    local identifiers = args.identifiers

    local ok, res = utils.preproc([[
@for i,v in pairs(compose) do
@   local type_declaration = type_declaration_if_variable_was_not_yet_defined(v[3], olist)
$(space)$(type_declaration)$(v[3]) = $(identifiers[ v[2] ]) * $(identifiers[ v[1] ]);
@end

]], {
        table = table,
        pairs = pairs,
        space = utils.gen_spaces('\t', idx),
        compose = compose,
        result_type = result_type,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const,
        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end


return gen_fnc_body_computations
