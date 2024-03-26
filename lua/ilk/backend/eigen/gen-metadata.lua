local keys = require("ilk.parser").keys
local backend_symbols = require('ilk.backend.eigen.backend-symbols')
local metat = require("ilk.common").metatypes
local common = require('ilk.backend.eigen.common')
local yaml = require('lyaml')

local eigen = require("ilk.backend.eigen.common")

local M = {}

local generate_output_type_name_pairs = function(program, context)
    local output_pairs = {}
    for k, v in pairs(program.source.metasignature.outputs) do
      local type = eigen.typeString(v.metatype, {nsQualified=true},context)
      table.insert(output_pairs, { type, v.defname } )
    end
    return output_pairs
end

local generate_input_type_name_pairs = function(program, context)
    local input_pairs = {}
    for k, v in pairs(program.source.metasignature.inputs) do
      local type = eigen.typeString(v.metatype, {nsQualified=true},context)
      table.insert(input_pairs, { type, v.defname } )
    end
    return input_pairs
end

local function translate_to_legacy_solver_type(type)
    if type == 'fk' then
        return 'forward'
    elseif type == 'ikpos' then
        return 'inverse'
    elseif type == 'ikvel' then
        return 'inverse'
    end
end

M.gen_metadata = function(context, path)
    local fd = io.open(path..'/metadata.yml', "w") or io.stdout
    local conf = { fd = fd, filename = fn }
    local metadata = {}

    for i, program in pairs(context.programs) do
        local entry = {}
        entry.robot_name = program.source.meta.robot_name
        entry.type  = translate_to_legacy_solver_type(program.source.meta.solver_type)
        if program.source.meta.solver_type == keys.solverKind.ik.velocity then
          entry.ik = { kind="vel"}
        elseif program.source.meta.solver_type == keys.solverKind.ik.position then
          entry.ik = { kind="pos"}
        end
        entry.outputs = {}
        for i,v in ipairs(program.source.metasignature.outputs) do
          table.insert( entry.outputs, {target=v.defname, otype=v.metatype})
        end
        entry.inputs = {}
        for i,v in ipairs(program.source.metasignature.inputs) do
          table.insert( entry.inputs, {target=v.defname, otype=v.metatype})
        end

        --entry.ops   = program.source.ops
        entry.olist = generate_output_type_name_pairs(program, context)
        entry.ilist = generate_input_type_name_pairs(program, context)
        entry.solver_name = program.source.meta.solver_id
--        local olist_no_jacobians = {}
--        for i,v in pairs(metadata_entry.olist) do
--            if string.match(v[1],constants.pose_type) then
--                olist_no_jacobians[#olist_no_jacobians+1] = v
--            end
--        end
--        metadata_entry.olist_without_jacobians = olist_no_jacobians

        metadata[entry.solver_name] = entry
    end

    local metadata_string = yaml.dump(metadata)
    fd:write(metadata_string)
    fd:close()
end



function sort_outputs(outputs)
    local sorted_outputs = {}
    for key,value in pairs(outputs) do
        sorted_outputs[#sorted_outputs + 1] = {
            name = key,
            otype = value.otype,
            usersort = value.usersort
        }
    end

    table.sort(sorted_outputs,
        function (a,b)
            return a.usersort < b.usersort
        end)
    return sorted_outputs
end

M.gen_metadata2 = function(context, path, programs)
    --local fd = io.open(path..robot_name..'_metadata.yml', "w") or io.stdout
    local fd = io.open(path..'/metadata2.yml', "w") or io.stdout
    local metadata = {}

    for i,parsed_program in pairs(context.programs) do
        local metadata_entry = {}

        local model_constants_prefix = 'mc'
        if parsed_program.metasignature ~= nil then
            for i,v in pairs(parsed_program.metasignature.inputs) do
                if (v.metatype == 'ModelConstants') then
                    model_constants_prefix = v.defname
                end
            end
        end

        local progMetaData = parsed_program.source.meta
        metadata_entry.solver_name = progMetaData.solver_id
        metadata_entry.type = translate_to_legacy_solver_type(progMetaData.solver_type)

        metadata_entry.ops = {
            solver_id = progMetaData.solver_id,
            type = translate_to_legacy_solver_type(progMetaData.solver_type),
            robot_name = progMetaData.robot_name,
            outputs = {},
            m_joint_local = {},
            jacobians = {},
            model_const = {},
            j_computes = {},
            identifiers = {},
            compose = {}
        }
        if progMetaData.solver_type ~= 'fk' then
            local knd = ""
            if progMetaData.solver_type == 'ikpos' then
                knd = "pos"
            elseif progMetaData.solver_type == 'ikvel' then
                knd = "vel"
            end
            metadata_entry.ops["ik"] = {
                target = "",
                kind = knd,
                op = "ik",
                reference = "",
                vectors = progMetaData.solver_specs.configSpace,
                fk = progMetaData.solver_specs.fkSolverID
            }

        end
        if parsed_program.ops ~= nil then
            for key,value in pairs(parsed_program.ops) do
                if value.op == keys.ops.pose_compose then
                    metadata_entry.ops.compose[#metadata_entry.ops.compose + 1] = {
                        value.arg1,
                        value.arg2,
                        value.res
                    }
                    local entry = {}
                    entry[value.res] = value.res
                    metadata_entry.ops.identifiers[#metadata_entry.ops.identifiers + 1] = entry
                elseif value.op == 'geom-jacobian' then
                    metadata_entry.ops.jacobians[#metadata_entry.ops.jacobians + 1] = {
                        name = value.name,
                        pose = value.pose,
                        op = value.op
                    }
                    local entry = {}
                    entry[value.name] = value.name
                    metadata_entry.ops.identifiers[#metadata_entry.ops.identifiers + 1] = entry
                elseif value.op == 'GJac-col' then
                    metadata_entry.ops.j_computes[#metadata_entry.ops.j_computes + 1] = {
                        joint_pose = value.joint_pose,
                        jac = value.jac,
                        op = value.op,
                        jtype = value.jtype,
                        col = value.col
                    }
                end
            end
        end

        if (progMetaData.outputs ~= nil) then
            local sorted_outputs = sort_outputs(progMetaData.outputs)

            for key,value in pairs(sorted_outputs) do
                metadata_entry.ops.outputs[#metadata_entry.ops.outputs + 1] = {
                    target = value.name, otype = value.otype, op = 'output'
                }
            end
        end

        if (parsed_program.model_poses ~= nil) then
            for key,value in pairs(parsed_program.model_poses.joint) do
                metadata_entry.ops.m_joint_local[#metadata_entry.ops.m_joint_local + 1] = {
                    op = 'model_T_joint_local',
                    name = key,
                    input = value.coordinate,
                    dir = value.dir,
                    jtype = value.jtype
                }
                local entry = {}
                entry[key] = key
                metadata_entry.ops.identifiers[#metadata_entry.ops.identifiers + 1] = entry
            end

            for key,value in pairs(parsed_program.model_poses.constant) do
                metadata_entry.ops.model_const[#metadata_entry.ops.model_const + 1] = key
                local entry = {}
                entry[key] = model_constants_prefix..'.'..key
                metadata_entry.ops.identifiers[#metadata_entry.ops.identifiers + 1] = entry
            end
        end



        metadata_entry.olist = {}
        if (progMetaData.outputs ~= nil) then
            local sorted_outputs = sort_outputs(progMetaData.outputs)

            for key,value in ipairs(sorted_outputs) do
                local olist_entry = {}
                if value.otype == 'jacobian' then
                    olist_entry = {
                        --common.typeString(key),
                        progMetaData.robot_name .. '::' .. 'Jacobian_t',
                        value.name
                    }
                elseif value.otype == 'pose' then
                    olist_entry = {
                        --common.typeString(key),
                        backend_symbols.namespaces[1] .. '::' .. backend_symbols.types[metat.pose],
                        value.name
                    }
                end
                metadata_entry.olist[#metadata_entry.olist + 1] = olist_entry
            end
        end

        local olist_no_jacobians = {}
        for i,v in pairs(metadata_entry.olist) do
            if v[1] ~= nil then
                if string.match(v[1],backend_symbols.types[metat.pose]) then
                    olist_no_jacobians[#olist_no_jacobians+1] = v
                end
            end
        end
        metadata_entry.olist_without_jacobians = olist_no_jacobians

        metadata[metadata_entry.solver_name] = metadata_entry
    end

    local metadata_string = yaml.dump(metadata)
    fd:write(metadata_string)
    fd:close()

end

return M
