local keys = require("ilk.parser").keys


local model_constant_cannot_be_requested_as_computation_output = function(program)
    if program.meta.outputs == nil or program.model_poses == nil then
        return ''
    end

    for i, v in pairs(program.meta.outputs) do
        for j, w in pairs(program.model_poses.constant) do
            if i == j then
                return i .. " was requested as a computation output, but it is a model constant."
            end
        end
    end
    return ''
end

local all_identifiers_used_should_be_generated_or_declared_as_constant = function(program)
    local created = {}
    local required = {}
    if program.meta.solver_type ~= 'fk' then
        return ''
    end

    if program.ops ~= nil then
        for i,v in pairs(program.ops) do
            if v.op == keys.ops.pose_compose then
                required[#required + 1] = v.arg1
                required[#required + 1] = v.arg2
                created[#created + 1] = v.res
            elseif v.op == 'geom-jacobian' then
                created[#created + 1] = v.name
                required[#required + 1] = v.pose
            end
        end
    end
    if program.model_poses ~= nil then
        if program.model_poses.constant ~= nil then
            for i,v in pairs(program.model_poses.constant) do
                created[#created + 1] = i
            end
        end
        if program.model_poses.joint ~= nil then
            for i,v in pairs(program.model_poses.joint) do
                created[#created + 1] = i
            end
        end
    end
    if program.meta.outputs ~= nil then
        for i,v in pairs(program.meta.outputs) do
            if v[keys.outputs.itemValueKeys.type] == keys.outputs.outtypes.pose then
              required[#required + 1] = i
            end
        end
    end



    for i,v in pairs(required) do
        local found = false
        for j,w in pairs(created) do
            if v == w then
                found = true
            end
        end
        if found == false then
            return v .. " is required, but it is neither a constant nor is generated during computations"
        end
    end

    return ''
end

local function run_sanity_checks(program, logger)
    local results = {}
    results[#results+1] = model_constant_cannot_be_requested_as_computation_output(program)
    results[#results+1] = all_identifiers_used_should_be_generated_or_declared_as_constant(program)
    for i,result in pairs(results) do
        if result ~= '' then
            logger.error(result)
            os.exit(1)
        end
    end
end

return run_sanity_checks