local constants = require('ilk.constants')

types_eigen = {
    Pose = {
        typedef = true,
        tname = constants.pose_type,
        views = { Position = 'position_v', Orientation = 'rot_m_v' }
    },
    Position = { typedef = true, tname = 'position_t' },
    Orientation = { typedef = true, tname = 'rot_m_t' }
}


--[[ TODO:
not smart generation of views --> generate only the one needed,
instead of all (aka output required from query)
--]]
local var_op_eigen = function(state, args, idx)
    local pose_template = [[

$(space)$(tname) $(varname);
$(space)$(varname).setIdentity();
@ if views then
@   for i,v in pairs(views) do
@    if i == 'Position' then
$(space)$(v) $(varname)_$(i) = eg_get_position($(varname));
@    elseif i == 'Orientation' then
$(space)$(v) $(varname)_$(i) = eg_get_rotation($(varname));
@    end
@   end
@ end

]]
    local templates = {
        Pose = pose_template
    }
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local tname = types_eigen[args[1]].tname
    local views = types_eigen[args[1]].views
    local varname = args[2]
    local typedef = types_gsl[args[1]].typedef
    local ok, res = utils.preproc(templates[args[1]],
        {
            table = table,
            pairs = pairs,
            space = utils.gen_spaces('\t', idx),
            tname = tname,
            varname = varname,
            typedef = typedef,
            views = views
        })
    if not ok then error(res) end
    state.vars[#state.vars + 1] = { varname = varname, tname = tname }
    fd:write(res)
end


--[[ TODO
  NOT OPTIMISED!
  Use of temporal as described in the program, but I could fetch a full
  chunk of 'compose' operations and concatenate them, considering the output
  requested by the query
--]]
local compose_op_eigen = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[$(space)$(args[3]) = $(args[1]) * $(args[2]);]], { table = table, space = utils.gen_spaces('\t', idx), args = args })
    if not ok then error(res) end
    fd:write(res)
end



--support only transformation_matrix(Pose)
local print_op_eigen = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
@ for i,v in pairs(args) do
$(space)std::cout << "---- $(v) ----" << std::endl;
$(space)std::cout << $(v) << std::endl;
$(space)std::cout << "--------------" << std::endl;
@ end
]], { table = table, args = args, pairs = pairs, space = utils.gen_spaces('\t', idx) })
    if not ok then error(res) end
    fd:write(res)
end


-- supports only transformation_matrix
local set_op_eigen = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0

    local values = model_values[args[2]]
    local ok, res = utils.preproc([[
$(space)eg_set_position($(target)_Position,$(values.p[1]),$(values.p[2]),$(values.p[3]));
$(space)eg_set_rotation($(target)_Orientation,$(values.r[1]),$(values.r[2]),$(values.r[3]),
$(space)    $(values.r[4]),$(values.r[5]),$(values.r[6]),
$(space)    $(values.r[7]),$(values.r[8]),$(values.r[9]));
]], { space = utils.gen_spaces('\t', idx), table = table, values = values, target = args[1] })
    if not ok then error(res) end
    fd:write(res)
end




local M = {}
M.types_eigen = types_eigen
M.compose_op_eigen = compose_op_eigen
M.var_op_eigen = var_op_eigen
M.print_op_eigen = print_op_eigen
M.set_op_eigen = set_op_eigen
return M

