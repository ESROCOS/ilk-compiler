local common = require('ilk.common')

local extract_model_values = function(args)
    local mvalues = {}
    local keys_in_order = common.extract_identifiers_in_order(args.model_values)
    for i, id in pairs(keys_in_order) do
        if args.model_values[id] then
            mvalues[id] = args.model_values[id]
        end
    end
    return mvalues, keys_in_order
end

local initialization_function_signature = function(sname)
    local signature = "mc_"..sname.."::mc_"..sname.."() {"
    return signature
end

local initialize_position_and_rotation = function(identifier, val, space)
    local ok, res = utils.preproc([[
$(space)$(identifier).setIdentity();
$(space)eg_set_position($(identifier),$(val.p[1]),$(val.p[2]),$(val.p[3]));
$(space)eg_set_rotation($(identifier),$(val.r[1]),$(val.r[2]),$(val.r[3]),
$(space)                     $(val.r[4]),$(val.r[5]),$(val.r[6]),
$(space)                     $(val.r[7]),$(val.r[8]),$(val.r[9]));
]], {
        table = table,
        space = space,
        val = val,
        identifier = identifier
    })
    if not ok then error(res) end
    return res
end

local function gen_model_cnst(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    -- pre-processing: fetching model values
    local mvalues, keys_in_order = extract_model_values(args)
    local function_signature = initialization_function_signature(args.sname)
    local ok, res = utils.preproc([[

$(space)$(function_signature)
@for i,v in pairs(keys_in_order) do
@   local initialization_code = initialize_position_and_rotation(v, values[v], space)
$(initialization_code)
@end
}

]], {
        table = table,
        pairs = pairs,
        function_signature = function_signature,
        initialize_position_and_rotation = initialize_position_and_rotation,
        space = utils.gen_spaces('\t', idx),
        values = mvalues,
        keys_in_order = keys_in_order,
        sname = args.sname
    })
    if not ok then error(res) end
    fd:write(res)
end


return gen_model_cnst