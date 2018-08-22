#! /usr/bin/env lua

utils = require('utils')
helpers = require('ilk.helpers')
op_backend = require('ilk.op-backend')
ilk_keywords = require('ilk.keywords')
ilk_parser = require('ilk.parser')
run_sanity_checks = require('ilk.sanity-checks')
inverse_kinematics = require('ilk.source-eigen.inverse-kinematics')


local function compute_function_name(ops)
    return ops.solver_id
end

------------- HEADER GENERATION -----------------------------
-- Routine to generete the header file
-- It contains all the functions generated by the program
function gen_header_main(path, fn, config)
    local fd = io.open(path .. fn .. config.fextension, "w") or io.stdout
    local conf = { fd = fd, filename = fn }

    local parsed_programs = {}
    local robot_name = ""
    for i,program in pairs(config.program) do
        local ops = ilk_parser.extract_operations(program)
        local olist = ilk_parser.generate_output_type_name_pairs(ops.outputs, ops.robot_name)
        local parsed_program = {}
        robot_name = ops.robot_name
        parsed_program.ops = ops
        parsed_program.olist = olist
        parsed_programs[#parsed_programs+1] = parsed_program
    end

    ------------------------------
    -- generation steps
    config.main.header(conf, { filename = fn }, 0, robot_name)
    config.main.model_config(conf,
        { sname = 'config', model_values = model_values }, 1)

    for i,parsed_program in pairs(parsed_programs) do
        local ops = parsed_program.ops
        local olist = parsed_program.olist
        if ops.type == ilk_keywords.solver_type.forward then
            config.main.gen_fnc_signature(conf, {
                fnc_name = compute_function_name(ops),
                sname = 'config',
                olist = olist,
                is_proto = true
            }, 1, ops.robot_name)
        elseif ops.type == ilk_keywords.solver_type.inverse then
            if ops.ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.velocity then
                inverse_kinematics.gen_ik_velocity_headers({ fd = fd }, ops, 0)
            elseif ops.ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.position then
                inverse_kinematics.gen_ik_position_headers({ fd = fd }, ops, 0)
            end
        end
    end
    config.main.footer(conf)
    fd:close()
end


function header_generator(config)
    local config = config
    config.main = {}
    if config.backend == 'eigen' then
        local eg_module = require('ilk.eigen')
        config.main = eg_module.header
        config.main.gen_fnc_signature = eg_module.source.gen_fnc_signature
        config.fextension = '.h'
    else
        helpers.errmsg("The header file generation is not supported with the selected backend (" .. config.backend .. ')'); os.exit(1)
    end

    return function(path, fn) gen_header_main(path, fn, config) end
end

function makefile_generator(config)
    local config = config
    local genm
    if config.backend == 'eigen' then
        genm = require('ilk.eigen').make.gen_makefile
    else
        helpers.errmsg("The generation of the makefile is not supported with the selected backend (" .. config.backend .. ')'); os.exit(1)
    end

    return function(path, args)
        local fd = io.open(path .. '/makefile', 'w')
        genm(fd, args)
        fd:close()
    end
end


function gen_test(path, fn, config)
    local fd = io.open(path .. fn .. '_test.cpp', "w") or io.stdout
    local conf = { fd = fd, filename = fn }
    config.main.header(conf)
    config.main.footer(conf)
    fd:close()
end


function extract_robot_name(programs)
    local robot_name = ""
    for i,program in pairs(programs) do
        local program_robot_name = ilk_parser.extract_robot_name(program)
        if robot_name == "" then
            robot_name = program_robot_name
        end
        if robot_name ~= program_robot_name then
            helpers.errmsg("ERROR: Robot name differs in ILK files! Found both '"..robot_name.."' and '"..program_robot_name.."'")
            os.exit(1)
        end
    end
    return robot_name
end


function gen_main(path, fn, config, model_values)
    local fd = io.open(path .. fn .. config.fextension, "w") or io.stdout
    local internal_config = { fd = fd }

    -- step 0: pre-process the full program
    -- grouping by operation

    local parsed_programs = {}
    local robot_name = extract_robot_name(config.program)
    for i,program in pairs(config.program) do
        local ops = ilk_parser.extract_operations(program)
        run_sanity_checks(ops.m_joint_local, ops.compose, ops.model_const, ops.outputs, ops.jacobians)
        local olist = ilk_parser.generate_output_type_name_pairs(ops.outputs, ops.robot_name)
        local parsed_program = {}
        parsed_program.ops = ops
        parsed_program.olist = olist
        parsed_programs[#parsed_programs+1] = parsed_program
    end

    ------------------------------
    -- generation steps
    config.main.header(internal_config, { filename = fn }, 0, robot_name)
    config.main.gen_model_cnst(internal_config, { model_values = model_values, sname = 'config' }, 0)
    for i,parsed_program in pairs(parsed_programs) do
        local ops = parsed_program.ops
        local olist = parsed_program.olist
        if ops.type == ilk_keywords.solver_type.forward then
            config.main.gen_fnc_signature(internal_config, {
                fnc_name = compute_function_name(ops),
                sname = 'config',
                olist = olist,
                is_proto = false --hardcoded, this is given
            }, 1, ops.robot_name)
            config.main.gen_joint_transform_local_variables(internal_config, ops.m_joint_local, ops.outputs, 1)
            config.main.gen_fnc_body_declarations(internal_config, ops.m_joint_local, 1)
            config.main.gen_fnc_body_computations(internal_config, ops, 1)
            config.main.gen_fnc_body_jacobians(internal_config, ops, 1)
            config.main.gen_fnc_body_close(internal_config)
        elseif ops.type == ilk_keywords.solver_type.inverse then
            if ops.ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.velocity then
                inverse_kinematics.gen_ik_velocity_computations(internal_config, ops, 0)
            elseif ops.ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.position then
                inverse_kinematics.gen_ik_position_computations(internal_config, ops, 0)
            end
        end
    end
    config.main.footer(internal_config, {}, 0)
    fd:close()
end

-- selects the generator from the backend
local function source_generator(config, model_values)
    local config = config
    local model_values = model_values

    config.main = {}
    if config.backend == 'gsl' then
        config.main = require('ilk.gsl')
        config.operations = op_backend.gsl
        config.fextension = '.c'
    elseif config.backend == 'eigen' then
        config.main = require('ilk.eigen').source
        config.operations = op_backend.eigen
        config.fextension = '.cpp'
    end

    return function(path, fn) gen_main(path, fn, config, model_values) end
end

local function test_generator(config)
    local config = config

    config.main = {}
    if config.backend == 'eigen' then
        config.main = require('ilk.eigen').test
        config.fextension = '.cpp'
    else
        helpers.errmsg("the backend is not supported"); os.exit(1);
    end

    return function(path, fn) gen_test(path, fn, config) end
end

---
-- Program enters here
-----------------------------------
local opttab = utils.proc_args(arg)

if #arg == 1 or opttab['-h'] then helpers.usage(); os.exit(1) end

local input_rjeq = false
local ilk
local sfile
local inputfolder = ""
local ilk_files = {}

if opttab['--indir'] then
    input_req = true
    inputfolder = opttab['--indir'][1] .. '/'
    sfile = 'model-constants.lua'

    lfs = require('lfs')
    for file in lfs.dir(inputfolder) do
        local ext = utils.split(file, '[\\.]')[2]
        if ext == 'ilk' then
            ilk_files[#ilk_files+1] = file
        end
    end
end


--if not input_req then
--    if not (opttab['-i'] and opttab['-i'][1]) then
--        helpers.errmsg("input missing (intermediate language source)  [-i]"); os.exit(1)
--    end
--
--    if not (opttab['-s'] and opttab['-i'][1]) then
--        helpers.errmsg("input missing (static model values)  [-s]"); os.exit(1)
--    end
--    ilk = opttab['-i'][1]
--    sfile = opttab['-s'][1]
--end

--outdir option bypass -o
outdir = false
local mflag = false
if opttab['--outdir'] and opttab['--outdir'][1] then
    outdir = opttab['--outdir'][1]
    mflag = outdir --makefile generated when outdir option is indicated
elseif not (opttab['-o'] and opttab['-o'][1]) then
    helpers.errmsg("missing output destination (generated C source) [-o]"); os.exit(1)
end

if not (opttab['-b'] and opttab['-b'][1]) then
    helpers.errmsg("missing backend, options are 'eigen', 'gsl'  [-b]"); os.exit(1)
end


if not outdir and opttab['-m'] then
    helpers.warnmsg("A makefile will be generated.")
    mflag = opttab['-m'][1]
end

backend = opttab['-b'][1]


local cflag = false
if (opttab['--compile']) then
    if not mflag then
        helpers.warnmsg("option --compile is disabled when makefile is not generated")
    else
        cflag = opttab['--compile'][1] or path .. output_file
        helpers.warnmsg("compile flag activated -- using " .. backend .. ", target " .. cflag)
    end
end

local silent = false
if (opttab['--silent']) then
    silent = true
end

--print(inputfolder..ilk)
local program = {}
local robot_name = ""
for i,ilk in pairs(ilk_files) do
    local chunk1 = loadfile(inputfolder .. ilk)
    local prog = chunk1()
    program[#program+1] = prog
    robot_name = prog.robot_name
end
local chunk2 = loadfile(inputfolder .. sfile)
model_values = chunk2() --TODO param me
-- RUN!

-- default output_file name -> .ilk entry
output_file = robot_name..'.cpp'
if not outdir then output_file = opttab['-o'][1] end
local source_outputfile = output_file
local output_file = utils.split(source_outputfile, '[\\.]')[1]
path = ""
-- generate outdir folder and path
if outdir then
    os.execute('mkdir -p ' .. outdir)
    path = outdir .. '/'
end


local cgen = source_generator({ backend = backend, program = program }, model_values)
cgen(path, output_file)
if not silent then
    helpers.succmsg(output_file .. '.cpp generated!')
end
local hgen = header_generator({ backend = backend, program = program })
hgen(path, output_file)
if not silent then
    helpers.succmsg(output_file .. '.h generated!')
end
local tgen = test_generator({ backend = backend, program = program })
tgen(path, output_file)
if not silent then
    helpers.succmsg(output_file .. '_test.cpp generated!')
end

if mflag then
    local mgen = makefile_generator({ backend = backend })
    mgen(outdir, { libname = output_file, execname = output_file, path_support = nil })
    if not silent then
        helpers.succmsg('makefile generated!')
    end
    os.execute('cp ' .. inputfolder .. '/robot-defs.h ' .. outdir .. '/robot-defs.h')
    if not silent then
        helpers.succmsg('copying robot definitions')
    end
end

-- Post-process utility
-- (here, installing + AADL wrapper)
if cflag then
    if backend == 'gsl' then
        if not silent then
            helpers.warnmsg('compiling...')
        end
        os.execute('gcc -fPIC -g -pedantic -Wall -o ' .. cflag .. ' -L`pwd`/blas/grc_gsl/ ' .. output_file .. ' -lgrc')
        if not silent then
            helpers.succmsg('           ...done!')
        end
    end
    if backend == 'eigen' then
        if not silent then
            helpers.warnmsg('compiling...')
        end
        --     os.execute('g++ $(pkg-config --cflags eigen3) -Ieigen/ -o '..cflag..' '..path..output_file..'_test.cpp '..path..source_outputfile..' eigen/joint-transforms.cpp')
        os.execute('cd ' .. outdir .. ' && make && cd ..')
        if not silent then
            helpers.succmsg('           ...done!')
        end
    end
end
