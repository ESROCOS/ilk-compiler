local lfs = require('lfs')
local keys = require('ilk.parser').keys

local commons   = require('ilk.backend.octave.common')
local sourcegen = require('ilk.backend.octave.source')
local ops_handlers = require('ilk.backend.octave.ops')

--- The actual code generator which creates the output files
local function generator(context, programs, config)
    local opath = config.path
    lfs.mkdir(opath)
    local fd = nil
    
    -- The file with the model constants
    fd = io.open(opath .. "/"  .. config.codeTweaks.model_constants_class_name .. ".m", "w") or io.stdout
    fd:write( sourcegen.model_constants(context, config) )
    fd:close()

    -- The files with the solvers, one file per solver as this is Octave
    for i, prog in ipairs(programs) do
        fd = io.open(opath .. "/" .. prog.source.meta.solver_id ..".m", "w") or io.stdout
        if prog.source.meta.solver_type == keys.solverKind.fk then
            sourcetext = sourcegen.solver_fk(prog, context, config)
            fd:write(sourcetext)
        else
            sourcetext = sourcegen.solver_ik(prog, context, config)
            fd:write(sourcetext)
        end
        fd:close()
      end
end

local function getGenerators(context, sourceTweakConfig)
    return ops_handlers, generator
end


local function augmentContext(oContext, sourcePrograms)
    local context = {
        outer = oContext,
        programs = {},
        -- add fields here as needed
    }
    for i, program in ipairs(sourcePrograms) do
        context.programs[i] = {
            source = program,
            signature = commons.programSignature(program, context)
            -- add fields here as needed
        }
    end
    return context, context.programs
end




return {
    augmentContext = augmentContext,
    getGenerators  = getGenerators,
    logger = logger
}
