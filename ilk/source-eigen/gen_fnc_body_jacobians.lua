local common = require('ilk.common')

local prismatic_or_revolute = function(v)
    if (v.jtype == 'revolute') then
        return 'revolute'
    else
        return 'prismatic'
    end
end

local call_arguments_if_revolute = function(v, identifiers, idx)
    if v.jtype ~= 'revolute' then
        return
    end
    local idx = idx or 0
    idx = idx + 1
    local ok, res = utils.preproc([[$(space)poi_$(v.jac),
$(space)eg_get_position($(identifiers[v.joint_pose])),]], {
        table = table,
        pairs = pairs,
        identifiers = identifiers,
        space = utils.gen_spaces('\t', idx),
        v = v
    })
    return res
end

local call_arguments_common = function(v, identifiers, idx)
    local idx = idx or 0
    idx = idx + 1
    local identifiers = identifiers
    local ok, res = utils.preproc([[$(space)eg_get_zaxis($(identifiers[v.joint_pose])),
$(space)$(v.jac).col($(v.col)));]], {
        table = table,
        pairs = pairs,
        identifiers = identifiers,
        space = utils.gen_spaces('\t', idx),
        v = v
    })
    if not ok then error(res) end
    return res
end

local gen_fnc_body_jacobian_computes = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local type_poi = 'position_t'
    local jacobians = args.jacobians
    local j_computes = args.j_computes
    local model_const = args.model_const
    local identifiers = args.identifiers
    --local olist = args.outputs
    local ok, res = utils.preproc([[
@for i,v in pairs(j_computes) do
$(space)geometricJacobianColumn_$(prismatic_or_revolute(v))(
$(call_arguments_if_revolute(v,identifiers,idx))$(call_arguments_common(v,identifiers,idx))
@end
]], {
        table = table,
        pairs = pairs,
        idx = idx,
        space = utils.gen_spaces('\t', idx),
        type_poi = type_poi,
        jacobians = jacobians,
        j_computes = j_computes,
        model_const = model_const,
        identifiers = identifiers,
        prismatic_or_revolute = prismatic_or_revolute,
        call_arguments_common = call_arguments_common,
        call_arguments_if_revolute = call_arguments_if_revolute
    })
    if not ok then error(res) end
    fd:write(res)
end


local gen_fnc_body_jacobian_pois = function(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local type_poi = 'position_t'
    local jacobians = args.jacobians
    local j_computes = args.j_computes
    local model_const = args.model_const
    local identifiers = args.identifiers
    --local model_const = args.model_const
    --local olist = args.outputs

    local ok, res = utils.preproc([[
@for i,v in pairs(jacobians) do
$(space)$(type_poi) poi_$(v.name) = eg_get_position($(identifiers[v.pose]));
@end
]], {
        table = table,
        pairs = pairs,
        space = utils.gen_spaces('\t', idx),
        type_poi = type_poi,
        jacobians = jacobians,
        j_computes = j_computes,
        model_const = model_const,
        identifiers = identifiers,
        prismaric_or_revolute = prismaric_or_revolute,
        call_arguments_if_revolute = call_arguments_if_revolute,
        call_arguments_common = call_arguments_common
--        compose = compose,
--        result_type = result_type,
--        model_const = model_const,
--        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end


local function gen_fnc_body_jacobians(config, args, idx)
    gen_fnc_body_jacobian_pois(config, args, idx)
    gen_fnc_body_jacobian_computes(config, args, idx)
end


return gen_fnc_body_jacobians

