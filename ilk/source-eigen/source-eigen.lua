--[[
  ilk-source-gen-eigen.lua
   Module for generating source code of an ilk
   program in cpp -- using Eigen3
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium.
   License: BSD2-clause
--]]

local utils = require('utils')
local constants = require('ilk.constants')

local M = {}

local function header(config, args, idx, robot_namespace)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
/*
  Source code library generated with ILK-GEN
  File generated on: $(datestr) (UTC)
*/
#include "$(filename).h"
#include <ilk/eigen/gjac.h>

using namespace $(backend_namespace);

]], {
        table = table,
        backend_namespace = constants.backend_namespace,
        robot_namespace = robot_namespace,
        datestr = os.date("!%c"),
        filename = args.filename
    })
    if not ok then error(res) end
    fd:write(res)
end

local function footer(config, args, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    fd:write([[
    
]])
end





M.header = header
M.footer = footer
M.gen_fnc_signature = require('ilk.source-eigen.gen_fnc_signature')
M.gen_model_cnst = require('ilk.source-eigen.gen_model_cnst')
M.gen_joint_transform_local_variables = require('ilk.source-eigen.gen_joint_transform_local_variables')
M.gen_fnc_body_declarations = require('ilk.source-eigen.gen_fnc_body_declarations')
M.gen_fnc_body_computations = require('ilk.source-eigen.gen_fnc_body_computations')
M.gen_fnc_body_jacobians = require('ilk.source-eigen.gen_fnc_body_jacobians')
M.gen_fnc_body_close = require('ilk.source-eigen.gen_fnc_body_close')

return M
