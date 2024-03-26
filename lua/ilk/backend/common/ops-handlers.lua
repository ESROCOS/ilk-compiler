local utils       = require('ilk.common')
local langcommons = require("ilk.backend.common.common")
local keys        = require("ilk.parser").keys

local vv = require("ilk.backend.common.velocity")
local aa = require("ilk.backend.common.acceleration")
local jj = require("ilk.backend.common.jacobian")


local function def_handler(op)
  langcommons.logger.warning("Unknown opcode '" .. op.op .. "'")
  return {}
end

local ops_handlers = function(program, backend)

  local arg_qd = langcommons.inputArgumentByType(program, utils.metatypes.jointVel)
  local arg_qdd= langcommons.inputArgumentByType(program, utils.metatypes.jointAcc)

  local handlers = {
    [keys.ops.pose_compose]    = function(op) return backend.poseCompose(program, op) end,
    [keys.ops.jacobian]        = function(op) return jj.geometricJacobian(program, op, backend) end,
    [keys.ops.jacobian_column] = function(op) return jj.geometricJacobianColumn(program, op, backend) end,
    [keys.ops.joint_vel_twist] = function(op) return vv.jointVelocityTwist(program, op, arg_qd, backend) end,
    [keys.ops.vel_compose]     = function(op) return vv.velocityCompose   (program, op, arg_qd, backend) end,
    [keys.ops.joint_acc_twist] = function(op)
                                   local ret = aa.jointAccelerationTwist(program, op, arg_qdd, backend)
                                   return ret
                                 end,
    [keys.ops.acc_compose] = function(op)
                               local ret= aa.accelerationCompose(program, op, arg_qd, arg_qdd, backend)
                               return ret
                             end
  }
  setmetatable(handlers, {__index=function() return def_handler end} )
  return handlers
end

local function textdump(t)
  local text = ""
  for k,v in pairs(t) do
    text = text .. k .. "=" .. v .. "  "
  end
  return text
end

local function dbg_compile_ops(program, backend)
  local handlers = ops_handlers(program, backend)

  for i,op in ipairs(program.source.ops) do
    print("\n", i,textdump(op), " :\n")
    local stuff = handlers[op.op](op)
    if type(stuff) == 'table' then
      for i,l in ipairs(stuff) do
        print(l)
      end
    else
      print( stuff )
    end
  end
end


local function translatedOpsIterator(program, backend)
  local hh  = ops_handlers(program, backend)
  local ops = program.source.ops
  local i = 0
  return function()
    i = i + 1
    local op = ops[i]
    if op ~= nil then
      return op, hh[ op.op ](op)
    else
      return nil
    end
  end
end

return {
  ops_handlers = ops_handlers,
  dbg_compile_ops = dbg_compile_ops,
  translate = translatedOpsIterator
}