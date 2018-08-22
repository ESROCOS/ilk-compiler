local types_gsl = {
    Pose = { typedef = false, tname = 'grc_transformation_matrix_gsl', init = 'grc_transformation_matrix_gsl_setup' },
    Position = { typedef = false, tname = 'struct grc_position_vector_gsl', init = 'grc_position_vector_gsl_setup' },
    Orientation = { typedef = false, tname = 'struct grc_rotation_matrix_gsl', init = 'grc_rotation_matrix_gsl_setup' }
}


local var_op_gsl = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local tname = types_gsl[args[1]].tname
    local varname = args[2]
    local typedef = types_gsl[args[1]].typedef
    local ok, res = utils.preproc([[
@ if typedef then
$(space)$(tname) $(varname);
@ else
$(space)struct $(tname) $(varnamej);
@ end
$(space)$(init)(&$(varname));
$(space)grc_transformation_matrix_gsl_init_identity(&$(varname));

]],
        {
            table = table,
            space = utils.gen_spaces('\t', idx),
            tname = tname,
            varname = varname,
            typedef = typedef,
            init = types_gsl[args[1]].init
        })
    if not ok then error(res) end
    state.vars[#state.vars + 1] = { varname = varname, tname = tname }
    fd:write(res)
end

local compose_op_gsl = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[$(space)grc_transformation_matrix_gsl_compose(&$(args[1]),&$(args[2]),&$(args[3]));]], { table = table, space = utils.gen_spaces('\t', idx), args = args })
    if not ok then error(res) end
    fd:write(res)
end


--support only transformation_matrix
local print_op_gsl = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
@ for i,v in pairs(args) do
$(space)for(unsigned int i=0;i<3;i++) {
$(space)  printf("%f ", ($(v).origin)->data[i]);
$(space)}
$(space)printf("\n");
@ end
]], { table = table, args = args, pairs = pairs, space = utils.gen_spaces('\t', idx) })
    if not ok then error(res) end
    fd:write(res)
end


-- supports only transformation_matrix
local set_op_gsl = function(state, args, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0

    local values = model_values[args[2]]
    local ok, res = utils.preproc([[
@ if not anyset then
$(space)struct grc_rotation_matrix_gsl *r = grc_rotation_matrix_gsl_construct_and_setup();
$(space)struct grc_position_vector_gsl *p = grc_position_vector_gsl_construct_and_setup();
@ end
$(space)grc_position_vector_gsl_init_from_xyz(p,$(values.p[1]),$(values.p[2]),$(values.p[3]));
$(space)grc_rotation_matrix_gsl_init_from_entries(r,$(values.r[1]),$(values.r[2]),$(values.r[3]),
$(space)    $(values.r[4]),$(values.r[5]),$(values.r[6]),
$(space)    $(values.r[7]),$(values.r[8]),$(values.r[9]));
$(space)grc_transformation_matrix_gsl_init_from_rotation_matrix_and_position_vector(&$(target),r,p);

]], { space = utils.gen_spaces('\t', idx), table = table, values = values, target = args[1], anyset = state.anyset })
    if not ok then error(res) end
    state.anyset = true
    fd:write(res)
end



-- Poor's man implementation, hypothesis <tname>_cleanup API
local cleanup_gsl = function(state, idx)
    local fd = state.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
@ for i,v in pairs(vars) do
$(space)$(v.tname)_cleanup(&$(v.varname));
@ end
]], { table = table, pairs = pairs, space = utils.gen_spaces('\t', idx), vars = state.vars })
    if not ok then error(res) end
    fd:write(utils.gen_spaces('\n\t', idx) .. '/* cleaning up...  */\n')
    fd:write(res)
end


local M = {}
M.types_gsl = types_gsl
M.compose_op_gsl = compose_op_gsl
M.var_op_gsl = var_op_gsl
M.print_op_gsl = print_op_gsl
M.set_op_gsl = set_op_gsl
M.cleanup_gsl = cleanup_gsl
return M

