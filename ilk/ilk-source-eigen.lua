--[[
  ilk-source-gen-eigen.lua
   Module for generating source code of an ilk
   program in cpp -- using Eigen3
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium.
   License: BSD2-clause
--]]

local utils = require('ilk.utils')

local M = {}


local function header(config,args,idx)
  local fd  = config.fd or io.stdout
  local idx = idx or 0
  local ok, res = utils.preproc([[
/*
  Source code library generated with ILK-GEN
  File generated on: $(datestr) (UTC)
*/
#include "$(filename).h"

]], {table=table, 
     datestr  = os.date("!%c"),
     filename = args.filename
     })
  if not ok then error(res) end
  fd:write(res)
end

local function footer(config,args,idx)
  local fd  = config.fd or io.stdout
  local idx = idx or 0
  fd:write([[
    
]])
end

local function gen_model_cnst(config,args,idx)
  local fd = config.fd or io.stdout
  local idx = idx or 0
  -- pre-processing: fetching model values
  local mvalues = {}
  for i, id in pairs(args.model_const) do
    if args.model_values[id] then
      mvalues[id] = args.model_values[id]
    end
  end
  local ok, res = utils.preproc([[

$(space)mc_$(sname)::mc_$(sname)() {
@for i,v in pairs(vars) do
$(space)$(v).setIdentity();
@local val = values[v]
$(space)eg_set_position($(v),$(val.p[1]),$(val.p[2]),$(val.p[3]));
$(space)eg_set_rotation($(v),$(val.r[1]),$(val.r[2]),$(val.r[3]),
$(space)                     $(val.r[4]),$(val.r[5]),$(val.r[6]),
$(space)                     $(val.r[7]),$(val.r[8]),$(val.r[9]));

@end
}

]], {table=table, pairs=pairs, space=utils.gen_spaces('\t',idx),
    vars=args.model_const, values=mvalues, sname=args.sname})
  if not ok then error(res) end
  fd:write(res)
end

local function gen_fnc_signature(config,args,idx)
  local fd  = config.fd or io.stdout
  local idx = idx or 0
  local ending = ' {'
  if args.is_proto then ending = ';' end
  -- TODO this part is hardcoded
--   local itypes = {"joint_state"}
  local olist = args.olist
  -----------
  local tmp = {}
  utils.foreach(function(v,k)
    tmp[#tmp+1] = table.concat(v,'& ')
  end, olist)
  local ostr = table.concat(tmp,', ')
  
  -- end hardcoded part
  local ok, res = utils.preproc([[
void $(fnc_name)(const mc_$(sname)& mc, const $(inputtype)& input, $(output))$(ends)
]] ,{table=table,space=utils.gen_spaces('\t',idx),
  fnc_name=args.fnc_name, sname=args.sname, inputtype="joint_state", output=ostr, ends=ending} )
  if not ok then error(res) end
  fd:write(res)
end

local gen_fnc_body = function(config,args,idx)
  -- args is list of op 'model_T_joint_local'
  local fd = config.fd or io.stdout
  local idx = idx or 0
  local ok, res = utils.preproc([[
@for i,v in pairs(vars) do
$(space)pose_t $(v.name);
@  if v.jtype == "revolute" then
@    if v.dir == "a_x_b" then
$(space)rot_z__a_x_b(input($(v.input)),$(v.name));
@    else
$(space)rot_z__b_x_a(input($(v.input)),$(v.name));
@    end
@  elseif v.jtype == "prismatic" then
@    if v.dir == "b_x_a" then
$(space)tr_z__b_x_a(input($(v.input)),$(v.name));
@    else
$(space)tr_z__a_x_b(input($(v.input)),$(v.name));
@    end
@  end  
@end


]], {table=table, pairs=pairs, space=utils.gen_spaces('\t',idx),
    vars=args})
  if not ok then error(res) end
  fd:write(res)
end

-- This handles the generation of the concrete computation
local gen_fnc_body2 = function(config,args,idx)
  local fd  = config.fd or io.stdout
  local idx = idx or 0
  local compose = args.compose
  local model_const = args.model_const
--       { 'j1_base', 'j2_link1', 'j3_link2' }
  local olist       = args.outputs --[[or {
    'link2_base', 'link3_base'
  }]]
  local is_const = function(id)
    for i,v in pairs(model_const) do      
      if v == id then return "mc." end
    end
  end
  local is_output = function(id)
    for i,v in pairs(olist) do
      if id == v then return true end
    end
    return false
  end
  local ok, res = utils.preproc([[
@for i,v in pairs(compose) do
@local isc2 = is_const(v[2])
@local isc1 = is_const(v[1])
@local lhs = space
@if not is_output(v[3]) then
@  lhs = lhs..'pose_t '
@end
$(lhs)$(v[3]) = $(isc2)$(v[2]) * $(isc1)$(v[1]);
@end

}
]], {table=table, pairs=pairs, space=utils.gen_spaces('\t',idx),
    compose=compose, is_output=is_output, is_const=is_const})
  if not ok then error(res) end
  fd:write(res)
end

M.header = header
M.footer = footer
M.gen_fnc_signature = gen_fnc_signature
M.gen_model_cnst    = gen_model_cnst
M.gen_fnc_body      = gen_fnc_body        
M.gen_fnc_body2     = gen_fnc_body2

return M
