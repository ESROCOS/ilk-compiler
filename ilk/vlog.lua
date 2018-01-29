--[[
    vlog.lua: a verbose logger utility
  Author: Enea Scioni
  email:  <enea.scioni@kuleuven.be>
  Inspired by Markus Klotzbuecher's UMF
--]]

ac    = require('ilk.ansicolors')
utils = require('ilk.utils')

local M = {}

local function concat_ctx(ctx,sep)
   local ret = ""
   for i,v in ipairs(ctx) do
      if string.match(v, "%[(%d+)]") then ret=ret..v else ret=ret..sep..v end
   end
   return ret
end

--- Add an error to the validation struct
-- @param validation structure
-- @param level: 'err', 'warn', 'info'
-- @param msg string message
local function add_msg(vres, level, msg)
   if not vres then return end
   if not (level=='err' or level=='warn' or level=='info') then
      error("add_msg: invalid level: " .. tostring(level))
   end
   local msgs   = vres.msgs
   local lvname = level
   msgs[#msgs+1] = {
     level   = level,
     context = concat_ctx(vres.context, '.'), 
     txt     = msg.txt,
     depth   = msg.depth or #vres.context
   }
   vres[level] = vres[level] + 1
   return vres
end

--- Push error message context.
-- @param vres validation result table
-- @param field current context in form of table field.
local function vres_push_context(vres, field)
   if not vres then return end
   vres.context=vres.context or {}
   if type(field) ~= 'string' then error("vres_push_context: field not a string but "..type(field)) end
   local depth = #vres.context
   vres.context[depth+1] = field
   return depth
end

--- Pop error message context.
-- @param vres validation result table
local function vres_pop_context(vres, old_depth)
   if not vres then return end
   vres.context=vres.context or {}
   local cur_depth = #vres.context
   if old_depth~=nil and old_depth ~= cur_depth - 1 then
      error("vres_pop_context: unbalanced context stack at "..table.concat(vres.context, '.')..
         ". should be "..ts(old_depth).." but is "..ts(cur_depth-1))
   end
   if #vres.context <= 0 then error("vres_pop_context with empty stack") end
   vres.context[#vres.context] = nil
end

-- Colorize a message
local function colorize(tag, msg, bright,colorflag)
  if not colorflag then return msg end
  if     tag=='info' then msg = ac.cyan(msg)--ac.blue(msg)
  elseif tag=='warn' then msg = ac.yellow(msg)
  elseif tag=='err'  then msg = ac.red(msg)
  elseif tag=='ctx'  then msg = ac.darkgray(msg) end
  if bright then msg = ac.bright(msg) end
  return msg
end

-- Find max depth in vres
local function vres_maxdepth(vres)
  local depth = 0
  utils.foreach(function(v,i) depth = utils.max(v.depth,depth) end, vres.msgs)
  return depth
end

-- Generate a string with a validation resume
local function vres_resume(vres)
  return tostring(vres.err) .. " errors, " .. tostring(vres.warn) .. " warnings, ".. tostring(vres.info) .. " informational messages."
end

-- Converts a single message into a string
-- @param message (table)
-- @param colorflag true for colorful messages (optional, default false)
-- @return string 
local function vres_msg2str(msg, colorflag)
  local colorflag = colorflag or false
  local lvname = msg.level
  if lvname == 'err' then lvname = 'err ' end --extra space, pretty print
  return colorize(msg.level, lvname.." @ ",true,colorflag)..colorize('ctx', msg.context..": ",false,colorflag)..colorize(msg.level,msg.txt,false,colorflag)
end

-- Filters out the results in the validation structure
-- @param filter
-- @return filtered results (table)
local function vres_filter(vres,filtername)
  if not filtername then return vres.msgs end
  
  local _filterfnc = function(flevel) return utils.filter(function(v,i)
    if v.level == flevel then return true end
    return false
  end, vres.msgs)
  end
  
  local _filterdepth = function(depth)
    return utils.filter(function(v,i)
      if v.depth == depth then return true end
      return false
    end, vres.msgs)
  end
  
  if filtername == 'inner' then return _filterdepth(vres_maxdepth(vres)) end
  return _filterfnc(filtername)
end

--- Print the validation results.
-- @param validation struct
-- @param filter_level (optional), filter by levels ('err','warn' or 'info') or by depth ('inner')
-- @return detailed resume of the validation struct
local function vres_print(vres,filter_level)
  local colorflag = vres.opt.color or false
  local str       = ""
  utils.foreach(function(msg)
    str = str..vres_msg2str(msg,colorflag)..'\n'
  end, vres_filter(vres,filter_level))
  if not filter_level then str=str..vres_resume(vres) end
  return str
end

-- Creates an empty validation struct
-- @return validation structure
local function create_vres(opt)
  local opt  = opt or { color=true }
  local vres = { msgs={}, err=0, warn=0, info=0, context={}, opt=opt}
  -- Enabling column object syntax
  vres.pop_context  = vres_pop_context
  vres.push_context = vres_push_context
  vres.add_msg      = add_msg
  vres.resume       = vres_resume
  vres.filter       = vres_filter
  vres.result       = vres_print
  vres.clear        = function(vres) vres.msgs={}; vres.err=0; vres.warn=0; vres.info=0; vres.context = {}; end
  local mt= getmetatable(vres) or {}
  mt.__tostring     = function() return vres_print(vres) end
  setmetatable(vres,mt)
  return vres
end


-- Exposing
M.create_vres  = create_vres
M.msg2str      = vres_msg2str
M.VERSION      = "0.0.1"

return M