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
@if path_support ~= nil then
CFLAGS=$(CFLAGS) -I$(path_support)
@end

LIB=lib$(libname).so

.PHONY: clean

all: $(execname) $(LIB)

$(LIB) : $(libname).cpp
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ -lilkeigenbackend

$(execname): $(LIB)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(execname) $(libname)_test.cpp $(LIB)

clean:
	rm -f $(execname) $(LIB)
]]

local mk_no_replace = {
    CC = '$(CC)',
    WFLAGS = '$(WFLAGS)',
    LDFLAGS = '$(LDFLAGS)',
    CFLAGS = '$(CFLAGS)',
    EFLAGS = '$(EFLAGS)',
    LIB = '$(LIB)'
}

local function gen_makefile(fd, args)
    local fd = fd or io.stdout
    local env = {
        table = table,
        path_support = args.path_support,
        libname = args.libname,
        execname = args.execname
    }
    utils.merge_tab(env, mk_no_replace)
    local ok, res = utils.preproc(mk_template, env)
    if not ok then error(res) end
    fd:write(res)
end


M.gen_makefile = gen_makefile

return M
