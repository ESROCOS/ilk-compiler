local logging = require('log')
local logger  = logging.new("warning", require('log.writer.console.color').new())
local ilkcommons = require('ilk.common')

local tplengine = require('template-text')


local function loadTemplate(body, env, included)
    local ok, loaded = tplengine.tload(body, {xtendStyle=true}, env, included)
    if not ok then
        print(table.concat(loaded, "\n"))
        error("Failed to load the template", 2)
    end
    return loaded
end

local function evalTemplate(loaded, ret_table)
    local ok, ret
    local do_return_table = ret_table or false
    ok, ret = loaded.evaluate({returnTable=do_return_table})
    if not ok then
        print(table.concat(ret, "\n"))
        error("Failed to evaluate the template", 2)
    end
    return ret
end

local function loadAndEvalTemplate(body, env, ret_table, included)
    local loaded = loadTemplate(body, env, included)
    local ret    = evalTemplate(loaded, ret_table)
    return ret
end

local formalParameter = function(metaparameter)
    return {
        meta = metaparameter,
        name = metaparameter.defname, -- we could change this behavior,
    }
end


local signature = function(program, context)
    local ret = {
        inputs  = {},
        outputs = {},
        name    = program.metasignature.defaultName, -- we could customize this
        meta    = program.metasignature,
    }
    local iparam_names = {}
    local oparam_names = {}
    for i,metaparam in ipairs(program.metasignature.inputs) do
        ret.inputs[i] = formalParameter(metaparam)
        iparam_names[i] = ret.inputs[i].name
    end
    for i,metaparam in ipairs(program.metasignature.outputs) do
        ret.outputs[i] = formalParameter(metaparam)
        oparam_names[i] = ret.outputs[i].name
    end

    ret.toString = function(opts)
        local opts = opts or {}
        if opts.call then
            if opts.args == nil then
                logger.error("Cannot generate the invoke-code without the name of the arguments")
            end
            -- TODO possibly, checks
            local args = table.concat(opts.args, ", ")
            return ret.name.."("..args..")"
        end
        return string.format("function [%s] = %s(%s)",
            table.concat(oparam_names,","), ret.name, table.concat(iparam_names,", ") )
    end
    return ret
end

local poseValueExpressionGenerator = function(program)
    local mc = nil
    for i,p in pairs(program.signature.inputs) do
        if p.meta.metatype == ilkcommons.metatypes.modelConsts then
            mc = p.name
        end
    end
    if mc==nil then
        error("could not find the ModelConstants parameter in the signature of program "..program.source.meta.solver_id)
    end
    return
    function(poseid)
        if program.source.model_poses.constant[poseid] ~= nil then
            return mc .. "." .. poseid
        else
            -- the pose of interest is not a model constant. We can only assume
            -- a variable for it is available in the local scope, and that such
            -- variable is named as the pose itself
            return poseid
        end
    end
end


ilkcompiler__octave_logger = logger

return {
    programSignature = signature,
    logger = logger,
    loadTemplate = loadTemplate,
    evalTemplate = evalTemplate,
    loadAndEvalTemplate = loadAndEvalTemplate,
    poseValueExpressionGenerator = poseValueExpressionGenerator,
}
