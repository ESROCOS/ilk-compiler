local backend = require("ilk.backend.eigen.backend-symbols")
local tpl     = require("ilk.template-text").template_eval

local logger = require('log').new(
  "trace",
  require('log.writer.console.color').new()
)

local M = {}

M.logger = logger


---
--- @param ids the list of identifiers corresponding to nested namespaces
M.namespaceUtils = function(ids)
  return {
    qualifier = function() return table.concat(ids, "::") end,
    open      = function()
                  local text = {}
                  for i,v in ipairs(ids) do
                    text[i] = "namespace "..v.." {"
                  end
                  return text
                end,
    close     = function()
                  local text = {}
                  for i,v in ipairs(ids) do text[i] = "}" end
                  return text
                end
  }
end

M.contextNS = function(context) return M.namespaceUtils({context.robotName}) end
M.backendNS = M.namespaceUtils( backend.namespaces )



local metat = require("ilk.common").metatypes
local typesMap = {
  [metat.jointState]  = "joint_state",
  [metat.jointVel]    = "joint_state", -- in C++/Eigen we use the same container for position and velocity
  [metat.jointAcc]    = "joint_state", -- and also for acceleration
  [metat.modelConsts] = "ModelConstants",
  [metat.jacobian]    = "Jacobian_t"
}

M.typeString = function(type, opts, context)
  if type==nil then error("Nil type passed to typeString()") end

  local opts = opts or {}
  local ret = typesMap[type]
  if ret~=nil then
    -- we have a robot-type
    if opts.nsQualified then
      if context==nil then
        error("Request for a qualified type string, but no context was given")
      end
      ret = context.namespace.qualifier().."::"..ret
    end
  else
    ret=backend.types[type]
    if opts.nsQualified then
      ret = M.backendNS.qualifier().."::"..ret
    end
  end

  return ret
end


-- We make the assumption that the signature will always appear "within" the
-- context, so that context types (ie robot types) do not need to be namespace
-- qualified. On the other hand, backend types (like pose) do need the namespace
-- (a more sophisticated strategy would take into account whether the source has
-- a 'using namespace' directive, which would make the backend-namespace
-- qualification also not necessary)


local function isBackendType(metatype)
  return (backend.types[metatype]~=nil)
end

local fpReferenceType = {
  input = function(valuetype) return "const "..valuetype.."&" end,
  output= function(valuetype) return           valuetype.."&" end
}

local formalParameter = function(metaparameter)
  local name = metaparameter.defname -- we could change this behavior
  local valuetype =
        function(opts, context)
          return M.typeString(metaparameter.metatype, opts, context)
        end
  return {
    meta    = metaparameter,
    name    = name,
    valuetype = valuetype,
    type    = function(opts,context)
                return fpReferenceType[metaparameter.direction]( valuetype(opts,context) )
              end
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

  local iargs = {}
  local oargs = {}
  for i,v in pairs(ret.inputs) do
    iargs[i] = v.type( {nsQualified=isBackendType(v.meta.metatype)}, context ).." "..v.name
  end
  for i,v in pairs(ret.outputs) do
    oargs[i] = v.type( {nsQualified=isBackendType(v.meta.metatype)}, context ).." "..v.name
  end
  local inputs = table.concat(iargs, ", ")
  local outputs= table.concat(oargs, ", ")

  local iCount = program.metasignature.iParsCount
  local oCount = program.metasignature.oParsCount
  ret.toString = function( opts )
                   if opts.call then
                     if opts.args == nil and (iCount+oCount) > 0 then
                       logger.error("Cannot generate the solver-call code without the name of the arguments")
                       error("Missing arguments list to generate solver invocation")
                     else
                       local count = 0
                       for _ in pairs(opts.args) do count=count+1 end
                       if count ~= (iCount+oCount) then
                         logger.warning("Wrong number of arguments for the generation of the solver invocation ("..
                                        (iCount+oCount) .. " expected, got " .. count .. ")")
                       end
                     end
                     return ret.name.."(".. table.concat(opts.args, ", ") ..")"
                   end
                   local ns = ""
                   local ending = ""
                   if opts.declaration then ending = ";"
                   else ns = context.namespace.qualifier().."::" end
                   return "void "..ns..ret.name.."("..inputs..", "..outputs..")"..ending
                end
  return ret
end


M.findProgramByID = function(id, context)
  for i,prog in ipairs(context.programs) do
    if prog.source.meta.solver_id == id then
      return prog, i
    end
  end
  return nil, nil
end


M.matrixBlockExpr = function (mx, columns, whichBlock)
 local ns = M.backendNS.qualifier().."::"
 local evalType = ns..backend.matrixT(3,columns)
 local expr = mx..".template block<3,"..columns..">("..ns..backend.coordsID[whichBlock].x..",0)"
 return {
   evalType = evalType,  expr = expr,
   defAndInit = function(localVar)
     return evalType.." "..localVar.." = "..expr..";"
   end
 }
end


M.signatureUtils = function(program, context)
  local signature = program.signature
  local ret = {}

  ret.localVarForEachArgument = function()
    local env = {
      signature = program.signature,
      context = context
      }
      local ok, res = tpl(
[[
@for i,inp in ipairs(signature.inputs) do
«inp.valuetype({nsQualified=true}, context)» «inp.name»;
@end
@for i,outp in ipairs(signature.outputs) do
«outp.valuetype({nsQualified=true}, context)» «outp.name»;
@end
]],  env, {returnTable=true, xtendStyle=true})
    if not ok then
      logger.error("In C++ code template evaluation: " .. res)
      error("Failure in text template evaluation. See the log for more details.")
    end
    return res
  end

  ret.allArgsList = function()
    local names = {}
    for i,name in signature.meta.inputNames() do
      table.insert(names, name)
    end
    for i,name in signature.meta.outputNames() do
      table.insert(names, name)
    end
    return names
  end

  return ret
end



return M

