--[[
    ILK-GEN
   Main module for Eigen backend support
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium
   License: BSD2-clause
--]]

local M = {
    source = require('ilk.source-eigen.source-eigen'),
    header = require('ilk.source-eigen.header-eigen'),
    test = require('ilk.source-eigen.test-eigen'),
    make = require('ilk.source-eigen.makefile-eigen'),
}

return M