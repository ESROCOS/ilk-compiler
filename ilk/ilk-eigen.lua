--[[
    ILK-GEN
   Main module for Eigen backend support
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium
   License: BSD2-clause
--]]

local M = {
  source = require('ilk.ilk-source-eigen'),
  header = require('ilk.ilk-header-eigen'),
  test   = require('ilk.ilk-test-eigen'),
  make   = require('ilk.ilk-makefile-eigen')
}

return M