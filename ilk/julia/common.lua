local backend = require("ilk.julia.backend-symbols")
local metat = require("ilk.common").metatypes

local logging = require('log')

local M = {}

M.logger = logging.new("warning", require('log.writer.console.color').new())


local formalParameter = function(metaparameter)
  local name = metaparameter.defname -- we could change this behavior
  return {
    meta    = metaparameter,
    name    = name,
  }
end


M.signature = function(program, context)
  local ret = { inputs={}, outputs={}, meta=program.metasignature }

  for i,metaparam in pairs(program.metasignature.inputs) do
    ret.inputs[i] = formalParameter(metaparam)
  end
  for i,metaparam in pairs(program.metasignature.outputs) do
    ret.outputs[i] = formalParameter(metaparam)
  end
  ret.name = program.metasignature.defaultName -- we could customize this

  local inputs = {}
  for i,v in pairs(ret.inputs) do
    inputs[i] = v.name
  end

  ret.toString = function( opts )
                   if opts.call then
                     if opts.args == nil then
                       M.logger.error("Cannot generate the call-code without the name of the arguments")
                     end
                     -- TODO possibly, checks
                     local args = table.concat(opts.args, ", ")
                     return ret.name.."("..args..")"
                   end
                   return "function "..ret.name.."("..table.concat(inputs,", ")..")"
  end
  return ret
end


M.returnStatement = function(program)
  local outputs = {}
  for i,v in ipairs(program.signature.outputs) do
    outputs[i] = v.name
  end
  return "return "..table.concat(outputs,", ")
end


M.matrixColumn = function(mx, colIndex)
  return "view(" .. mx .. ", :," .. colIndex+1 .. ")"
end

M.matrixBlockExpr = function (mx, columns, whichBlock)
  local startRow = backend.coordsID[whichBlock].x
  local expr = "view(" .. mx .. ", " .. startRow .. ":" .. startRow .. "+2, 1:" ..columns.. ")"
  return {
    expr = expr,
    defAndInit = function(localVar)
                   return localVar .. " = " .. expr
                 end
  }
end

M.defaultLocalDefinition = function(program, context, parameter)
  local type = parameter.meta.metatype
  if (type==metat.jointState) or (type==metat.jointVel) then
    return parameter.name .. " = " .. context.qualifiedBackendFunction("matrixInit").."("..program.source.meta.joint_space_size..",".. 1 ..")"
  else
    return parameter.name .. " = nothing"
  end
end

return M

