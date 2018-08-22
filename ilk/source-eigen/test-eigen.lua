--[[
  ilk-test-eigen.lua
   Module for generating tests on a generated ILK
   program in cpp -- using Eigen3
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium.
   License: BSD2-clause
--]]

local utils = require('utils')
local M = {}

local function test_header_eigen(config, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
#include <iostream>
#include "$(filename).h"

]], { table = table, filename = config.filename })
    if not ok then error(res) end
    fd:write(res)
end

local function test_footer_eigen(config, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[

int main(int argc, char** argv) {
//  exec_query();
  return 0;
}
]], { table = table })
    if not ok then error(res) end
    fd:write(res)
end

-- Exposing
M.header = test_header_eigen
M.footer = test_footer_eigen

return M
