local constants = require('ilk.constants')

local extract_forward_operations = function(program)
    local m_joint_local = {}
    local compose = {}
    local model_const = {}
    local outputs = {}
    local jacobians = {}
    local jacobian_computes = {}
    local solver_id = program[ilk_keywords.configuration.solver_id]
    local robot_name = program[ilk_keywords.configuration.robot_name]
    local identifiers = {}

    for i, v in pairs(program) do
        if v.op == ilk_keywords.op.joint_local then
            m_joint_local[#m_joint_local + 1] = v
            identifiers[v.name] = v.name
        elseif v.op == ilk_keywords.op.compose then --be aware, they are ordered
            compose[#compose + 1] = v.args
            identifiers[v.args[3]] = v.args[3]
        elseif v.op == ilk_keywords.op.output then
            outputs[#outputs + 1] = v
            identifiers[v.target] = v.target
        elseif v.op == ilk_keywords.op.model_constant then
            model_const = v.args
            for j, w in pairs(v.args) do
                identifiers[w] = constants.const_prefix.."."..w
            end
        elseif v.op == ilk_keywords.op.jacobian_poi then
            jacobians[#jacobians + 1] = v
            identifiers[v.name] = v.name
        elseif v.op == ilk_keywords.op.jacobian_compute then
            jacobian_computes[#jacobian_computes + 1] = v
        end
    end

    local ops = {}
    ops.m_joint_local = m_joint_local
    ops.compose = compose
    ops.model_const = model_const
    ops.outputs = outputs
    ops.jacobians = jacobians
    ops.j_computes = jacobian_computes
    ops.solver_id = solver_id
    ops.robot_name = robot_name
    ops.identifiers = identifiers
    ops.type = ilk_keywords.solver_type.forward
    return ops
end


local extract_inverse_operations = function(program)
    local solver_id = program[ilk_keywords.configuration.solver_id]
    local robot_name = program[ilk_keywords.configuration.robot_name]
    local identifiers = {}
    local ik = {}
    for i, v in pairs(program) do
        if v.op == ilk_keywords.op.inverse_kinematics then
            ik = v
            identifiers[v.target] = v.target
            identifiers[v.reference] = v.reference
        end
    end

    local ops = {}
    ops.solver_id = solver_id
    ops.robot_name = robot_name
    ops.identifiers = identifiers
    ops.ik = ik
    ops.outputs = {}
    ops.type = ilk_keywords.solver_type.inverse
    return ops
end


local extract_operations = function(program)
    local solver_type = program[ilk_keywords.configuration.solver_type]
    if (solver_type == ilk_keywords.solver_type.inverse) then
        local ops = extract_inverse_operations(program)
        return ops
    elseif (solver_type == ilk_keywords.solver_type.forward) then
        local ops = extract_forward_operations(program)
        return ops
    end
end


local generate_output_type_name_pairs = function(outputs, robot_name)
    local output_pairs = {}
    for i, v in pairs(outputs) do
        local output_type = constants.backend_namespace..'::'..constants.pose_type
        if v.otype == 'jacobian' then
            output_type = robot_name..'::'..'t_'..v.target
        end
        output_pairs[#output_pairs + 1] = { output_type, v.target }
    end
    return output_pairs
end

local extract_robot_name = function(program)
    return program[ilk_keywords.configuration.robot_name]
end

M = {}
M.extract_operations = extract_operations
M.generate_output_type_name_pairs = generate_output_type_name_pairs
M.extract_robot_name = extract_robot_name

return M
