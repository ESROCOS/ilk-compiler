--[[
  ilk-makefile-eigen.lua
   Module for creating a makefile to compile
   the generated kinematic utilities -- using Eigen3
   Enea Scioni, <enea.scioni@kuleuven.be>
   2018, KU Leuven, Belgium.
   License: BSD2-clause
--]]

local utils = require('utils')

local M = {}

local mk_template = [[
CC=g++
LDFLAGS=-Wl,-R,./

eigen_cflags:= ${shell pkg-config --cflags eigen3}
    CFLAGS=-Wall -pedantic -std=c++11 ${eigen_cflags} -O2 -DEIGEN_NO_DEBUG
CFLAGS_DBG=-Wall -pedantic -std=c++11 ${eigen_cflags} -O0 -g -ftest-coverage -fprofile-arcs

@if path_support ~= nil then
    CFLAGS=$(CFLAGS) -I$(path_support)
CFLAGS_DBG=$(CFLAGS_DBG) -I$(path_support)
@end

LIB=lib$(libname).so

.PHONY: clean

all: $(LIB)

$(LIB) : $(libname).cpp
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ -lilkeigenbackend

tests : $(exe_list)
tests_dbg : $(exe_dbg_list)

@for i,testfile in pairs(testfiles) do
$(testfile) : $(testfile).cpp $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB) -lilkeigenbackend
@end

@for i,testfile in pairs(testfiles) do
$(testfile)_dbg : $(testfile).cpp $(libname).cpp
	$(CC) $(CFLAGS_DBG) -o $@ $^ -lilkeigenbackend
@end

clean:
	rm -f $(LIB)
]]

local mk_no_replace = {
    CC = '$(CC)',
    WFLAGS = '$(WFLAGS)',
    LDFLAGS = '$(LDFLAGS)',
    CFLAGS = '$(CFLAGS)',
    CFLAGS_DBG = '$(CFLAGS_DBG)',
    EFLAGS = '$(EFLAGS)',
    LIB = '$(LIB)'
}

local function gen_makefile(fd, args)
    local fd = fd or io.stdout
    local exe_list = ''
    local exe_dbg_list = ''
    table.sort(args.testfiles)
    for i,testfile in pairs(args.testfiles) do
        exe_list = exe_list .. " " .. testfile
        exe_dbg_list = exe_dbg_list .. " " .. testfile .. "_dbg"
    end
    local env = {
        table = table,
        pairs = pairs,
        path_support = args.path_support,
        libname = args.libname,
        exe_list = exe_list,
        exe_dbg_list = exe_dbg_list,
        testfiles = args.testfiles,
    }
    utils.merge_tab(env, mk_no_replace)
    local ok, res = utils.preproc(mk_template, env)
    if not ok then error(res) end
    fd:write(res)
end


M.gen_makefile = gen_makefile

return M
