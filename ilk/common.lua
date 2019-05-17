logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

local keys = require("ilk.parser").keys

local M = {}

M.metatypes = {
  pose        = "pose",
  position3d  = "position",
  orient3d    = "orientation",
  twist       = "twist",
  velocity    = "twist",
  linVel3d    = "linearVelocity",
  angVel3d    = "angularVelocity",
  jointState  = "jointState",
  jointVel    = "jointVelocity",
  modelConsts = "ModelConstants",
  jacobian    = "jacobian",
  axisAngle   = "axisAngle",
  ikCfg       = "ikCfg",
  ikDbg       = "ikDbg"
}


local positionTypes = { location   = M.metatypes.position3d,
                        orientation= M.metatypes.orient3d,
                        pose       = M.metatypes.pose }
local velocityTypes = { location   = M.metatypes.linVel3d,
                        orientation= M.metatypes.angVel3d,
                        pose       = M.metatypes.twist }


local function metaArgsSetterCommon(program, signature)
  signature.inputs  = {}
  signature.outputs = {}

  signature.inputs[1] = { defname="mc", metatype=M.metatypes.modelConsts, direction="input" }
  signature.inputs[2] = { defname="q" , metatype=M.metatypes.jointState, direction="input" }

  local requireJVelocity = false
  for k,v in pairs(program.meta.outputs) do
    -- Note how we are relying on 'v.otype' to be matching one of the _keys_ of
    -- the local metatypes table. Anywhere else we should always use the
    -- _value_ of an entry of the metatypes table.
    signature.outputs[ v.usersort ] = { defname=k, metatype=M.metatypes[v.otype], direction="output" }
    if v.otype == "twist" or v.otype == "velocity" then
      requireJVelocity = true
    end
  end
  if requireJVelocity then
    signature.inputs[3] = { defname="qd", metatype=M.metatypes.jointVel, direction="input"}
  end
end

local metaArgsSetter = {
  [keys.solverKind.fk] =
    function(program, signature)
      metaArgsSetterCommon(program, signature)
    end,

  [keys.solverKind.ik.velocity] =
    function(program, signature)
      metaArgsSetterCommon(program, signature)
      table.insert(signature.inputs,
        { defname="v" , metatype=velocityTypes[program.meta.solver_specs.configSpace], direction="input" })
    end,

  [keys.solverKind.ik.position] =
    function(program, signature)
      signature.inputs = {
        [1] = { defname="mc" , metatype=M.metatypes.modelConsts, direction="input" },
        [2] = { defname="cfg", metatype=M.metatypes.ikCfg,       direction="input" },
      }
      local cfgSpace = program.meta.solver_specs.configSpace
      if cfgSpace == "pose" then
        table.insert(signature.inputs,
          { defname="desp" , metatype=positionTypes.location, direction="input" })
        table.insert(signature.inputs,
          { defname="deso" , metatype=positionTypes.orientation, direction="input" })
      else
        table.insert(signature.inputs,
          { defname="desired" , metatype=positionTypes[cfgSpace], direction="input" })
      end
      table.insert(signature.inputs,
          { defname="q_guess" , metatype=M.metatypes.jointState, direction="input" })

      -- set the outputs by hand, because we have more actual output arguments than
      -- those modeled by program.meta.outputs; specifically, in addition to the
      -- joint state, we also have the ik_dbg log
      signature.outputs = {
          [1] = { defname="q_ik" , metatype=M.metatypes.jointState, direction="output" },
          [2] = { defname="dbg" , metatype=M.metatypes.ikDbg, direction="output" }
      }
    end
}


local function argByType(args, type)
  for i,a in ipairs(args) do
    if a.metatype==type then return i,a end
  end
    return nil,nil
end

local function argNames(args)
  local i = 0
  return function()
           i = i+1
           local v = args[i]
           if v then
             return i, v.defname
           end
         end
end

M.metaSignature = function(program)
  local sign = {}
  sign.defaultName = program.meta.solver_id

  metaArgsSetter[ program.meta.solver_type ](program, sign)

  sign.inputByType  = function(type) return argByType(sign.inputs , type) end
  sign.outputByType = function(type) return argByType(sign.outputs, type) end
  sign.inputNames   = function() return argNames(sign.inputs)  end
  sign.outputNames  = function() return argNames(sign.outputs) end

  local count = 0
  for _ in pairs(sign.inputs) do count=count+1 end
  sign.iParsCount = count
  count = 0
  for _ in pairs(sign.outputs) do count=count+1 end
  sign.oParsCount = count
  return sign
end


M.findProgramByID = function(id, context)
  local getSolverId = nil
  if context["outer"] ~= nil then
    -- we have a language-specific context
    getSolverId = function(prog) return prog.source.meta.solver_id end
  else
    -- we have the generic context
    getSolverId = function(prog) return prog.meta.solver_id end
  end
  for i,prog in pairs(context.programs) do
    if getSolverId(prog) == id then
      return prog, i
    end
  end
  return nil, nil
end


M.alphabPairs = function(t)
  local sortedKeys = t.__sortedKeys
  if sortedKeys == nil then
    sortedKeys = {}
    for k,_ in pairs(t) do
      table.insert(sortedKeys, k)
    end
    table.sort(sortedKeys) -- sort the values alphabetically
                           -- but the values here are the keys of the original table
    t.__sortedKeys = sortedKeys
  end
  local i = 1
  return function()
           local key = sortedKeys[i]
           i = i+1
           return key, t[key]
         end
end

M.operationsByKey = function(program, opkey)
  local count = 0
  local retops= {}
  for  i,op in ipairs(program.ops) do
    if op.op == opkey then
      table.insert(retops, op)
      count = count + 1
    end
  end
  return retops,count
end


local fail = function()
  error("Errors encountered; check the log for details")
end

local tpl = require('ilk.template-text').template_eval
M.tplEval_failOnError = function(template, env, opts)
  local ok,code = tpl(template, env, opts or {})
  if not ok then
    logger.error("In the evaluation of a text template: " .. code)
    fail()
  end
  return code
end

return M


