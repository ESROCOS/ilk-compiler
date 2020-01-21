local utils = require('ilk.common')


local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)


local fail = function()
  error("In the backend-independent functions; check the log for details")
end


local metaToConcrete = function(concreteContainer, metaElement)
  for k,v in pairs(concreteContainer) do
    if k == "meta" then
      if v == metaElement then
        return concreteContainer
      end
    else
      if type(v) == "table" then
        local res = metaToConcrete(v, metaElement)
        if res ~= nil then
          return res
        end
      end
    end
  end
  return nil
end


local argumentFromMeta = function(program, metaArgument)
  for i,v in ipairs(program.signature.inputs) do
    if v.meta == metaArgument then return v end
  end
  for i,v in ipairs(program.signature.outputs) do
    if v.meta == metaArgument then return v end
  end
  return nil
end


local inputArgumentByType = function(program, type)
  local i, meta_arg = program.source.metasignature.inputByType( type )
  if i ~= nil then
    local arg = program.signature.inputs[i]
    if arg.meta ~= meta_arg then
      logger.warning("Inconsistent signature of program " .. program.source.meta.solver_id)
    end
    return arg
  end
  return nil
end


local outputArgumentByType = function(program, type)
  local i,meta_arg = program.source.metasignature.outputByType( type )
  if meta_arg ~= nil then
    return argumentFromMeta(program, meta_arg) -- could be nil
  end
  return nil
end


return {
  logger = logger,
  fail   = fail,
  inputArgumentByType = inputArgumentByType,
  outputArgumentByType = outputArgumentByType
}