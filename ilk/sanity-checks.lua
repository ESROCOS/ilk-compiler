local model_constant_cannot_be_requested_as_computation_output = function(outputs, model_const)
    if outputs == nil then
        return ''
    end
    for i, v in pairs(outputs) do
        for j, w in pairs(model_const) do
            if v.target == w then
                return v.target .. " was requested as a computation output, but it is a model constant."
            end
        end
    end
    return ''
end

local all_identifiers_used_should_be_generated_or_declared_as_constant = function(m_joint_local, compose, outputs, model_const, jacobians)
    local created = {}
    local required = {}
    if m_joint_local ~= nil then
        for i,v in pairs(m_joint_local) do
            created[#created + 1] = v.name
        end
    end
    if compose ~= nil then
        for i,v in pairs(compose) do
            created[#created + 1] = v[3]
            required[#required + 1] = v[1]
            required[#required + 1] = v[2]
        end
    end
    if model_const ~= nil then
        for i,v in pairs(model_const) do
            created[#created + 1] = v
        end
    end
    if outputs ~= nil then
        for i,v in pairs(outputs) do
            required[#required + 1] = v.target
        end
    end
    if jacobians ~= nil then
        for i,v in pairs(jacobians) do
            created[#created + 1] = v.name
            required[#required + 1] = v.pose
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

local function run_sanity_checks(m_joint_local, compose, model_const, outputs, jacobians)
    local results = {}
    results[#results+1] = model_constant_cannot_be_requested_as_computation_output(outputs, model_const)
    results[#results+1] = all_identifiers_used_should_be_generated_or_declared_as_constant(m_joint_local, compose, outputs, model_const, jacobians)
    for i,result in pairs(results) do
        if result ~= '' then
            helpers.errmsg(result)
            os.exit(1)
        end
    end
end

return run_sanity_checks